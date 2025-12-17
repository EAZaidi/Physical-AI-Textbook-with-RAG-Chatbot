"""Agent tools for RAG retrieval.

Implements search_textbook as an async function tool for the OpenAI agent.
"""

import hashlib
from typing import List, Dict, Any
import asyncio

from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, Filter, FieldCondition, MatchValue

from agent_api.config import settings
from agent_api.models import RetrievalResult, ChunkMetadata
from agent_api.confidence import calculate_confidence_metrics
from agent_api.utils.retry import openai_retry
from agent_api.utils.circuit_breaker import CircuitBreaker

# Initialize clients
openai_client = OpenAI(api_key=settings.openai_api_key)
qdrant_client = QdrantClient(url=settings.qdrant_url, api_key=settings.qdrant_api_key)

# Initialize circuit breaker for Qdrant (T045)
qdrant_circuit_breaker = CircuitBreaker(
    failure_threshold=5,
    timeout=60,
    success_threshold=2,
    name="Qdrant"
)


async def search_textbook(query: str, top_k: int = None, similarity_threshold: float = None) -> Dict[str, Any]:
    """Search the textbook for relevant information.
    
    This is the ONLY source of information the agent can use. Must be called
    before answering any question.
    
    Args:
        query: The user's question
        top_k: Number of chunks to retrieve (default from config)
        similarity_threshold: Minimum similarity score (default from config)
        
    Returns:
        Dictionary with retrieval results and quality metrics
    """
    if top_k is None:
        top_k = settings.top_k_default
    if similarity_threshold is None:
        similarity_threshold = settings.similarity_threshold_default
    
    # Generate query embedding with exponential backoff retry (T044)
    @openai_retry
    def create_embedding():
        return openai_client.embeddings.create(
            input=query,
            model=settings.openai_embedding_model
        )

    embedding_response = await asyncio.to_thread(create_embedding)
    query_vector = embedding_response.data[0].embedding

    # Search Qdrant with circuit breaker protection (T045)
    def search_qdrant():
        return qdrant_client.search(
            collection_name=settings.qdrant_collection,
            query_vector=query_vector,
            limit=top_k,
            score_threshold=similarity_threshold
        )

    search_results = await asyncio.to_thread(
        qdrant_circuit_breaker.call,
        search_qdrant
    )
    
    # Extract results
    chunks = []
    scores = []
    metadata_list = []
    chunk_embeddings = []

    for hit in search_results:
        chunks.append(hit.payload.get("text", ""))
        scores.append(hit.score)

        # Extract chunk embedding for diversity calculation (T036)
        if hasattr(hit, 'vector') and hit.vector:
            chunk_embeddings.append(hit.vector)

        # Create metadata
        metadata = ChunkMetadata(
            chapter=hit.payload.get("chapter", "Unknown"),
            section=hit.payload.get("section", "Unknown"),
            url=hit.payload.get("url", ""),
            chunk_index=hit.payload.get("chunk_index", 0),
            content_hash=hashlib.sha256(hit.payload.get("text", "").encode()).hexdigest()[:16]
        )
        metadata_list.append(metadata)

    # Calculate multi-metric confidence assessment (T034, T035, T036)
    confidence_metrics = calculate_confidence_metrics(
        scores=scores,
        num_chunks=len(chunks),
        chunk_embeddings=chunk_embeddings if chunk_embeddings else None
    )

    # Legacy fields for backward compatibility
    average_score = confidence_metrics.average_similarity
    has_sufficient_context = confidence_metrics.should_answer
    
    # Build result
    result = RetrievalResult(
        chunks=chunks,
        scores=scores,
        metadata=metadata_list,
        average_score=average_score,
        has_sufficient_context=has_sufficient_context,
        query_embedding=query_vector
    )
    
    # Return as dict for agent tool response
    return {
        "chunks": chunks,
        "scores": scores,
        "average_score": average_score,
        "has_sufficient_context": has_sufficient_context,
        "num_results": len(chunks),
        "metadata": [
            {
                "chapter": m.chapter,
                "section": m.section,
                "url": m.url,
                "chunk_index": m.chunk_index
            }
            for m in metadata_list
        ],
        # Multi-metric confidence assessment (T034-T037)
        "confidence_metrics": {
            "average_similarity": confidence_metrics.average_similarity,
            "min_similarity": confidence_metrics.min_similarity,
            "max_similarity": confidence_metrics.max_similarity,
            "num_chunks": confidence_metrics.num_chunks,
            "chunk_diversity": confidence_metrics.chunk_diversity,
            "confidence_level": confidence_metrics.confidence_level,
            "should_answer": confidence_metrics.should_answer
        }
    }
