"""Multi-metric confidence scoring for RAG Agent API.

Implements advanced confidence assessment using 5 metrics instead of single average.
"""

from typing import List, Tuple
import numpy as np

from agent_api.models import ConfidenceMetrics
from agent_api.config import settings


def calculate_chunk_diversity(embeddings: List[List[float]]) -> float:
    """Calculate semantic diversity of retrieved chunks (T036).
    
    Diversity = 1.0 - mean(pairwise_similarities)
    Higher diversity = chunks cover different aspects of the topic.
    
    Args:
        embeddings: List of chunk embedding vectors
        
    Returns:
        Diversity score between 0.0 (identical) and 1.0 (completely different)
    """
    if len(embeddings) < 2:
        return 0.0
    
    # Convert to numpy array
    embeds = np.array(embeddings)
    
    # Calculate pairwise cosine similarities
    pairwise_sims = []
    for i in range(len(embeds)):
        for j in range(i + 1, len(embeds)):
            # Cosine similarity
            dot_product = np.dot(embeds[i], embeds[j])
            norm_i = np.linalg.norm(embeds[i])
            norm_j = np.linalg.norm(embeds[j])
            
            if norm_i > 0 and norm_j > 0:
                similarity = dot_product / (norm_i * norm_j)
                pairwise_sims.append(similarity)
    
    if not pairwise_sims:
        return 0.0
    
    # Diversity = 1 - average pairwise similarity
    avg_pairwise_sim = np.mean(pairwise_sims)
    diversity = 1.0 - avg_pairwise_sim
    
    # Clamp to [0, 1]
    return max(0.0, min(1.0, diversity))


def calculate_confidence_metrics(
    scores: List[float],
    num_chunks: int,
    chunk_embeddings: List[List[float]] = None
) -> ConfidenceMetrics:
    """Calculate multi-metric confidence assessment (T034).
    
    Uses 5 metrics instead of single average:
    1. Average similarity: Mean score
    2. Min similarity: Weakest match (detects poor retrievals)
    3. Max similarity: Best match
    4. Num chunks: How many results
    5. Chunk diversity: Semantic variance
    
    Args:
        scores: Similarity scores from Qdrant
        num_chunks: Number of retrieved chunks
        chunk_embeddings: Optional embeddings for diversity calculation
        
    Returns:
        ConfidenceMetrics with level and should_answer decision
    """
    if not scores or num_chunks == 0:
        return ConfidenceMetrics(
            average_similarity=0.0,
            min_similarity=0.0,
            max_similarity=0.0,
            num_chunks=0,
            chunk_diversity=0.0,
            confidence_level="insufficient",
            should_answer=False
        )
    
    # Calculate metrics
    average_similarity = sum(scores) / len(scores)
    min_similarity = min(scores)
    max_similarity = max(scores)
    
    # Calculate diversity if embeddings provided
    chunk_diversity = 0.0
    if chunk_embeddings and len(chunk_embeddings) > 1:
        chunk_diversity = calculate_chunk_diversity(chunk_embeddings)
    
    # Determine confidence level using thresholds (T035)
    confidence_level, should_answer = determine_confidence_level(
        average_similarity,
        min_similarity,
        num_chunks,
        chunk_diversity
    )
    
    return ConfidenceMetrics(
        average_similarity=average_similarity,
        min_similarity=min_similarity,
        max_similarity=max_similarity,
        num_chunks=num_chunks,
        chunk_diversity=chunk_diversity,
        confidence_level=confidence_level,
        should_answer=should_answer
    )


def determine_confidence_level(
    avg_sim: float,
    min_sim: float,
    count: int,
    diversity: float
) -> Tuple[str, bool]:
    """Determine confidence level from multi-metric assessment (T035).
    
    Confidence Levels:
    - HIGH (>0.8): avg >= 0.85, min >= 0.75, count >= 3
    - MEDIUM (0.5-0.8): avg >= 0.75, min >= 0.65, count >= 2
    - LOW (<0.5): avg >= 0.60, count >= 1 (REFUSE to answer)
    - INSUFFICIENT (0): Below thresholds (REFUSE)
    
    Args:
        avg_sim: Average similarity
        min_sim: Minimum similarity
        count: Number of chunks
        diversity: Chunk diversity score
        
    Returns:
        Tuple of (confidence_level, should_answer)
    """
    # HIGH confidence
    if (avg_sim >= settings.confidence_threshold_high and
        min_sim >= 0.45 and
        count >= 3):
        return "high", True

    # MEDIUM confidence
    elif (avg_sim >= settings.confidence_threshold_medium and
          min_sim >= 0.35 and
          count >= 2):
        return "medium", True
    
    # LOW confidence - refuse to answer (conservative approach)
    elif (avg_sim >= settings.confidence_threshold_low and 
          count >= 1):
        return "low", False  # Refuse for safety
    
    # INSUFFICIENT - definitely refuse
    else:
        return "insufficient", False


def apply_adaptive_thresholds(
    base_metrics: ConfidenceMetrics,
    query_type: str = "factual"
) -> ConfidenceMetrics:
    """Apply query-type-aware threshold adjustments (T037).
    
    Different query types may require different confidence thresholds.
    
    Args:
        base_metrics: Base confidence metrics
        query_type: Type of query (factual, conceptual, procedural)
        
    Returns:
        Adjusted confidence metrics
    """
    # For now, use base metrics
    # Future enhancement: adjust thresholds based on query type
    # - Factual: stricter thresholds (current)
    # - Conceptual: slightly relaxed (allow lower min_similarity)
    # - Procedural: require higher diversity (step-by-step needs multiple sources)
    
    return base_metrics


def get_confidence_explanation(metrics: ConfidenceMetrics) -> str:
    """Generate human-readable confidence explanation (T037).
    
    Args:
        metrics: Confidence metrics
        
    Returns:
        Explanation string
    """
    if metrics.confidence_level == "high":
        return (
            f"High confidence answer based on {metrics.num_chunks} highly relevant sources "
            f"(avg similarity: {metrics.average_similarity:.2f})."
        )
    elif metrics.confidence_level == "medium":
        return (
            f"Medium confidence answer based on {metrics.num_chunks} relevant sources "
            f"(avg similarity: {metrics.average_similarity:.2f}). "
            f"Limited information available in textbook."
        )
    elif metrics.confidence_level == "low":
        return (
            f"Low confidence - only {metrics.num_chunks} marginally relevant sources found "
            f"(avg similarity: {metrics.average_similarity:.2f}). Cannot provide reliable answer."
        )
    else:
        return (
            "Insufficient information in textbook to answer this question. "
            "No relevant sources found."
        )
