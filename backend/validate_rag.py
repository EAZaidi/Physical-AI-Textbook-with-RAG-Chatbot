#!/usr/bin/env python3
"""
RAG Pipeline Validation Script
Feature: 002-rag-retrieval-validation

Purpose: Validate that stored embeddings can be reliably retrieved and matched
to relevant book content for RAG use. Tests semantic search, metadata integrity,
relevance metrics, and performance under load.

Usage:
    python validate_rag.py                          # Run all validation phases
    python validate_rag.py --phase P1               # Run specific phase only
    python validate_rag.py --output report.json     # Save detailed JSON report
"""

import argparse
import hashlib
import os
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import cohere
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct

# Load environment variables
load_dotenv()


# ============================================================================
# Data Models
# ============================================================================

@dataclass
class Query:
    """Represents a test search query."""
    text: str
    top_k: int = 5
    query_type: str = "specific"  # specific, broad, paraphrase, edge


@dataclass
class SearchResult:
    """A single chunk retrieved from Qdrant."""
    similarity_score: float
    content: str
    url: str
    title: str
    chunk_index: int
    timestamp: str
    content_hash: str


@dataclass
class TestCase:
    """Complete test scenario with ground truth and evaluation."""
    query: Query
    expected_criteria: Dict
    actual_results: List[SearchResult] = field(default_factory=list)
    relevance_labels: List[int] = field(default_factory=list)
    precision_at_k: float = 0.0
    rank_of_best: Optional[int] = None


@dataclass
class ValidationReport:
    """Aggregated validation results across all test cases."""
    timestamp: str
    total_queries: int
    avg_precision_at_5: float = 0.0
    mrr: float = 0.0
    avg_latency_ms: float = 0.0
    p95_latency_ms: float = 0.0
    p99_latency_ms: float = 0.0
    metadata_completeness_rate: float = 0.0
    hash_validation_pass_rate: float = 0.0
    test_cases: List[TestCase] = field(default_factory=list)
    summary: str = ""
    issues: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict:
        """Convert ValidationReport to dictionary for JSON serialization."""
        return {
            "timestamp": self.timestamp,
            "total_queries": self.total_queries,
            "metrics": {
                "avg_precision_at_5": self.avg_precision_at_5,
                "mrr": self.mrr,
                "avg_latency_ms": self.avg_latency_ms,
                "p95_latency_ms": self.p95_latency_ms,
                "p99_latency_ms": self.p99_latency_ms,
                "metadata_completeness_rate": self.metadata_completeness_rate,
                "hash_validation_pass_rate": self.hash_validation_pass_rate,
            },
            "test_cases": [
                {
                    "query": {
                        "text": tc.query.text,
                        "top_k": tc.query.top_k,
                        "query_type": tc.query.query_type,
                    },
                    "expected_criteria": tc.expected_criteria,
                    "actual_results": [
                        {
                            "similarity_score": r.similarity_score,
                            "content": r.content,
                            "url": r.url,
                            "title": r.title,
                            "chunk_index": r.chunk_index,
                            "timestamp": r.timestamp,
                            "content_hash": r.content_hash,
                        }
                        for r in tc.actual_results
                    ],
                    "relevance_labels": tc.relevance_labels,
                    "precision_at_k": tc.precision_at_k,
                    "rank_of_best": tc.rank_of_best,
                }
                for tc in self.test_cases
            ],
            "summary": self.summary,
            "issues": self.issues,
        }


# ============================================================================
# Test Queries
# ============================================================================

TEST_QUERIES = [
    # Specific technical queries (US1 - P1)
    Query("How do ROS 2 nodes communicate?", 5, "specific"),
    Query("What is a URDF joint?", 5, "specific"),
    Query("How do I create a Gazebo world file?", 5, "specific"),
    Query("What is a ROS 2 launch file?", 5, "specific"),
    Query("How does Isaac Sim work?", 5, "specific"),

    # Broad conceptual queries (US1 - P1)
    Query("How do humanoid robots work?", 5, "broad"),
    Query("What is perception in robotics?", 5, "broad"),
    Query("What are Vision-Language-Action models?", 5, "broad"),

    # Paraphrase variants (US1 - P1)
    Query("ROS 2 message passing", 5, "paraphrase"),
    Query("Robot messaging systems", 5, "paraphrase"),

    # Module-specific queries (US1 - P1)
    Query("Unity Robotics simulation setup", 5, "specific"),
    Query("NVIDIA Isaac Sim getting started", 5, "specific"),

    # Edge cases (US4 - P4)
    Query("", 5, "edge"),  # Empty query
    Query("asdfghjkl", 5, "edge"),  # Nonsense query
    Query("quantum computing algorithms", 5, "edge"),  # Out of domain
]


# ============================================================================
# Client Initialization
# ============================================================================

def initialize_clients():
    """Initialize Cohere and Qdrant clients from environment variables."""
    cohere_api_key = os.getenv("COHERE_API_KEY")
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not cohere_api_key:
        raise ValueError("COHERE_API_KEY not found in environment variables")
    if not qdrant_url:
        raise ValueError("QDRANT_URL not found in environment variables")
    if not qdrant_api_key:
        raise ValueError("QDRANT_API_KEY not found in environment variables")

    cohere_client = cohere.Client(cohere_api_key)
    qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

    return cohere_client, qdrant_client


def verify_collection(qdrant_client: QdrantClient, collection_name: str = "rag_embedding"):
    """Verify Qdrant collection exists and retrieve collection info."""
    try:
        collection_info = qdrant_client.get_collection(collection_name)
        vector_count = collection_info.points_count
        print(f"Collection '{collection_name}' found")
        print(f"Total vectors: {vector_count}")
        return True
    except Exception as e:
        print(f"Error: Collection '{collection_name}' not found")
        print(f"Details: {e}")
        return False


# ============================================================================
# User Story 1: Basic Semantic Search Validation
# ============================================================================

def generate_query_embedding(cohere_client: cohere.Client, query_text: str) -> List[float]:
    """Generate query embedding using Cohere embed-english-v3.0.

    Args:
        cohere_client: Initialized Cohere client
        query_text: Query string to embed

    Returns:
        List of embedding values (1024-dimensional vector)

    Raises:
        Exception: If Cohere API call fails
    """
    try:
        response = cohere_client.embed(
            texts=[query_text],
            model="embed-english-v3.0",
            input_type="search_query"  # Optimized for query-document matching
        )
        return response.embeddings[0]
    except Exception as e:
        raise Exception(f"Cohere embedding generation failed: {e}")


def semantic_search(
    qdrant_client: QdrantClient,
    query_embedding: List[float],
    collection_name: str = "rag_embedding",
    limit: int = 5
) -> List[SearchResult]:
    """Perform semantic search using Qdrant query_points method.

    Args:
        qdrant_client: Initialized Qdrant client
        query_embedding: Query vector (1024-dimensional)
        collection_name: Qdrant collection name
        limit: Number of results to retrieve (top-K)

    Returns:
        List of SearchResult objects with scores and metadata

    Raises:
        Exception: If Qdrant search fails
    """
    try:
        results = qdrant_client.query_points(
            collection_name=collection_name,
            query=query_embedding,
            limit=limit
        )

        search_results = []
        for point in results.points:
            search_results.append(SearchResult(
                similarity_score=point.score,
                content=point.payload.get("content", ""),
                url=point.payload.get("url", ""),
                title=point.payload.get("title", ""),
                chunk_index=point.payload.get("chunk_index", -1),
                timestamp=point.payload.get("timestamp", ""),
                content_hash=point.payload.get("content_hash", "")
            ))

        return search_results
    except Exception as e:
        raise Exception(f"Qdrant semantic search failed: {e}")


def display_query_results(query_index: int, total_queries: int, query: Query, results: List[SearchResult], latency_ms: float):
    """Display query results to console with formatting.

    Args:
        query_index: Current query number (1-indexed)
        total_queries: Total number of queries
        query: Query object
        results: List of SearchResult objects
        latency_ms: Query latency in milliseconds
    """
    print(f"[{query_index}/{total_queries}] Query: \"{query.text}\"")
    print(f"  Query Type: {query.query_type}")
    print(f"  Top {len(results)} results retrieved in {latency_ms:.0f}ms")
    print()

    for i, result in enumerate(results, 1):
        print(f"  Result {i}:")
        print(f"    Score: {result.similarity_score:.4f}")
        print(f"    Content: {result.content[:150]}{'...' if len(result.content) > 150 else ''}")
        print(f"    Source: {result.title}")
        print(f"    URL: {result.url}")
        print()


def get_relevance_labels(query: Query, results: List[SearchResult]) -> List[int]:
    """Prompt user for manual relevance labels for each result.

    Args:
        query: Query object
        results: List of SearchResult objects

    Returns:
        List of relevance labels (0=Not Relevant, 1=Relevant, 2=Highly Relevant)
    """
    print("  Manual Relevance Labeling:")
    print("  Enter 0 = Not Relevant, 1 = Relevant, 2 = Highly Relevant")
    print()

    labels = []
    for i, result in enumerate(results, 1):
        while True:
            try:
                label = input(f"  Result {i} relevance (0/1/2): ").strip()
                label_int = int(label)
                if label_int in [0, 1, 2]:
                    labels.append(label_int)
                    break
                else:
                    print("    Invalid input. Please enter 0, 1, or 2")
            except ValueError:
                print("    Invalid input. Please enter 0, 1, or 2")
            except EOFError:
                # Handle non-interactive mode - default to 0
                labels.append(0)
                break

    return labels


def display_relevance_summary(labels: List[int]):
    """Display summary of relevance labels.

    Args:
        labels: List of relevance labels
    """
    label_names = {0: "Not Relevant", 1: "Relevant", 2: "Highly Relevant"}
    label_counts = {0: 0, 1: 0, 2: 0}

    for label in labels:
        label_counts[label] += 1

    print("  Relevance Summary:")
    for label_val in [2, 1, 0]:
        count = label_counts[label_val]
        print(f"    {label_names[label_val]}: {count}/{len(labels)}")

    relevant_count = label_counts[1] + label_counts[2]
    print(f"  Total Relevant: {relevant_count}/{len(labels)}")
    print()


# ============================================================================
# User Story 2: Metadata Integrity Verification
# ============================================================================

def validate_metadata_completeness(result: SearchResult) -> bool:
    """Check if all 6 required metadata fields are non-null.

    Args:
        result: SearchResult object

    Returns:
        True if all fields are non-null, False otherwise
    """
    return all([
        result.url,
        result.title,
        result.chunk_index is not None and result.chunk_index >= 0,
        result.content,
        result.timestamp,
        result.content_hash
    ])


def validate_content_hash(result: SearchResult) -> bool:
    """Recompute SHA256 hash and compare to stored hash.

    Args:
        result: SearchResult object

    Returns:
        True if recomputed hash matches stored hash, False otherwise
    """
    computed_hash = hashlib.sha256(result.content.encode('utf-8')).hexdigest()
    return computed_hash == result.content_hash


def validate_timestamp_format(timestamp: str) -> bool:
    """Validate ISO 8601 timestamp format.

    Args:
        timestamp: Timestamp string

    Returns:
        True if valid ISO 8601 format, False otherwise
    """
    try:
        datetime.fromisoformat(timestamp.replace('Z', '+00:00'))
        return True
    except (ValueError, AttributeError):
        return False


def validate_url_format(url: str) -> bool:
    """Validate URL format.

    Args:
        url: URL string

    Returns:
        True if valid URL format, False otherwise
    """
    return url.startswith('http://') or url.startswith('https://')


def compute_metadata_completeness_rate(results: List[SearchResult]) -> float:
    """Compute percentage of results with all fields non-null.

    Args:
        results: List of all SearchResult objects

    Returns:
        Float between 0.0 and 1.0
    """
    if not results:
        return 1.0

    complete_count = sum(1 for r in results if validate_metadata_completeness(r))
    return complete_count / len(results)


def compute_hash_validation_pass_rate(results: List[SearchResult]) -> float:
    """Compute percentage where recomputed hash matches stored hash.

    Args:
        results: List of all SearchResult objects

    Returns:
        Float between 0.0 and 1.0
    """
    if not results:
        return 1.0

    pass_count = sum(1 for r in results if validate_content_hash(r))
    return pass_count / len(results)


# ============================================================================
# User Story 3: Relevance Quality Assessment
# ============================================================================

def compute_precision_at_k(relevance_labels: List[int], k: int) -> float:
    """Calculate precision@K = (relevant results in top-K) / K.

    Args:
        relevance_labels: List of relevance labels (0, 1, 2)
        k: Cutoff (e.g., 5 for precision@5)

    Returns:
        Float between 0.0 and 1.0
    """
    if not relevance_labels or k == 0:
        return 0.0

    relevant_count = sum(1 for label in relevance_labels[:k] if label >= 1)
    return relevant_count / k


def find_rank_of_best(relevance_labels: List[int]) -> Optional[int]:
    """Find position of first result with label == 2 (Highly Relevant).

    Args:
        relevance_labels: List of relevance labels (0, 1, 2)

    Returns:
        1-indexed rank of best result, or None if no highly relevant results
    """
    for i, label in enumerate(relevance_labels, 1):
        if label == 2:
            return i
    return None


def compute_mrr(test_cases: List[TestCase]) -> float:
    """Calculate Mean Reciprocal Rank across all queries.

    Args:
        test_cases: List of TestCase objects

    Returns:
        Float between 0.0 and 1.0
    """
    if not test_cases:
        return 0.0

    reciprocal_ranks = []
    for tc in test_cases:
        if tc.rank_of_best is not None:
            reciprocal_ranks.append(1.0 / tc.rank_of_best)
        else:
            reciprocal_ranks.append(0.0)

    return sum(reciprocal_ranks) / len(test_cases)


# ============================================================================
# User Story 4: End-to-End Pipeline Stress Testing
# ============================================================================

def compute_latency_percentiles(latencies: List[float]) -> Dict[str, float]:
    """Compute latency percentiles (p50, p95, p99).

    Args:
        latencies: List of latency values in milliseconds

    Returns:
        Dictionary with p50, p95, p99 values
    """
    if not latencies:
        return {"p50": 0.0, "p95": 0.0, "p99": 0.0}

    import numpy as np
    return {
        "p50": float(np.percentile(latencies, 50)),
        "p95": float(np.percentile(latencies, 95)),
        "p99": float(np.percentile(latencies, 99))
    }


def parse_arguments():
    """Parse command-line arguments for validation script."""
    parser = argparse.ArgumentParser(
        description="RAG Pipeline Validation - Verify semantic search quality and metadata integrity",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                          Run all validation phases
  %(prog)s --phase P1               Run Phase 1 only (Basic Semantic Search)
  %(prog)s --phase all              Run all phases (default)
  %(prog)s --output results.json    Save detailed JSON report
        """
    )

    parser.add_argument(
        "--output",
        type=str,
        help="Path to save JSON validation report (optional)"
    )

    parser.add_argument(
        "--phase",
        type=str,
        choices=["P1", "P2", "P3", "P4", "all"],
        default="all",
        help="Which validation phase to run (default: all)"
    )

    return parser.parse_args()


def main():
    """Main entry point for RAG validation script."""
    args = parse_arguments()

    print("=" * 70)
    print("RAG Pipeline Validation")
    print("Feature: 002-rag-retrieval-validation")
    print("=" * 70)
    print(f"Phase: {args.phase}")
    if args.output:
        print(f"Output: {args.output}")
    print("=" * 70)
    print()

    # Initialize clients
    try:
        print("Initializing clients...")
        cohere_client, qdrant_client = initialize_clients()
        print("Cohere client: Connected")
        print(f"Qdrant URL: {os.getenv('QDRANT_URL')}")
        print()

        # Verify Qdrant collection
        print("Verifying Qdrant collection...")
        if not verify_collection(qdrant_client):
            print("\nError: Qdrant collection 'rag_embedding' not found")
            print("Please run the ingestion pipeline first (001-rag-ingestion-pipeline)")
            return 1
        print()

    except Exception as e:
        print(f"\nError initializing clients: {e}")
        return 1

    # Run validation for requested phase
    print(f"Running validation with {len(TEST_QUERIES)} test queries...")
    print("=" * 70)
    print()

    test_cases = []
    latencies = []

    for i, query in enumerate(TEST_QUERIES, 1):
        try:
            # Measure end-to-end latency
            start_time = time.time()

            # Generate query embedding
            if not query.text:  # Handle empty query edge case
                print(f"[{i}/{len(TEST_QUERIES)}] Query: (empty string)")
                print(f"  Query Type: {query.query_type}")
                print(f"  Skipping empty query (edge case)")
                print()
                continue

            query_embedding = generate_query_embedding(cohere_client, query.text)

            # Perform semantic search
            results = semantic_search(qdrant_client, query_embedding, limit=query.top_k)

            # Calculate latency
            end_time = time.time()
            latency_ms = (end_time - start_time) * 1000
            latencies.append(latency_ms)

            # Display results
            display_query_results(i, len(TEST_QUERIES), query, results, latency_ms)

            # Get relevance labels (manual input)
            labels = get_relevance_labels(query, results)

            # Display relevance summary
            display_relevance_summary(labels)

            # Compute metrics for this test case (US3)
            precision_at_k = compute_precision_at_k(labels, query.top_k)
            rank_of_best = find_rank_of_best(labels)

            # Store test case with computed metrics
            test_case = TestCase(
                query=query,
                expected_criteria={},
                actual_results=results,
                relevance_labels=labels,
                precision_at_k=precision_at_k,
                rank_of_best=rank_of_best
            )
            test_cases.append(test_case)

        except Exception as e:
            print(f"  Error processing query: {e}")
            print()
            continue

    # Compute aggregate metrics
    print("=" * 70)
    print("Computing Validation Metrics...")
    print("=" * 70)
    print()

    # Collect all results for metadata validation (US2)
    all_results = []
    for tc in test_cases:
        all_results.extend(tc.actual_results)

    # User Story 2: Metadata Integrity
    metadata_completeness_rate = compute_metadata_completeness_rate(all_results)
    hash_validation_pass_rate = compute_hash_validation_pass_rate(all_results)

    print(f"Metadata Completeness: {metadata_completeness_rate:.1%}")
    print(f"Hash Validation Pass Rate: {hash_validation_pass_rate:.1%}")
    print()

    # User Story 3: Relevance Quality
    avg_precision_at_5 = sum(tc.precision_at_k for tc in test_cases) / len(test_cases) if test_cases else 0.0
    mrr = compute_mrr(test_cases)

    # Count queries meeting precision threshold
    queries_meeting_threshold = sum(1 for tc in test_cases if tc.precision_at_k >= 0.80)
    query_success_rate = queries_meeting_threshold / len(test_cases) if test_cases else 0.0

    print(f"Average Precision@5: {avg_precision_at_5:.3f}")
    print(f"Mean Reciprocal Rank (MRR): {mrr:.3f}")
    print(f"Queries Meeting Threshold (>=0.80): {queries_meeting_threshold}/{len(test_cases)} ({query_success_rate:.1%})")
    print()

    # User Story 4: Latency Metrics
    percentiles = compute_latency_percentiles(latencies)
    avg_latency = sum(latencies) / len(latencies) if latencies else 0.0

    print(f"Average Latency: {avg_latency:.1f}ms")
    print(f"p50 Latency: {percentiles['p50']:.1f}ms")
    print(f"p95 Latency: {percentiles['p95']:.1f}ms")
    print(f"p99 Latency: {percentiles['p99']:.1f}ms")
    print()

    # Pass/Fail Determination
    print("=" * 70)
    print("VALIDATION SUMMARY")
    print("=" * 70)

    issues = []

    # Check success criteria
    check_precision = query_success_rate >= 0.80  # 80% of queries meet precision@5 >= 0.80
    check_mrr = mrr >= 0.70
    check_metadata = metadata_completeness_rate == 1.0
    check_hash = hash_validation_pass_rate == 1.0
    check_latency = percentiles['p95'] < 2000.0

    print(f"Query Success Rate: {query_success_rate:.1%} (target: >= 80%) {'PASS' if check_precision else 'FAIL'}")
    print(f"MRR: {mrr:.3f} (target: >= 0.70) {'PASS' if check_mrr else 'FAIL'}")
    print(f"Metadata Completeness: {metadata_completeness_rate:.1%} (target: 100%) {'PASS' if check_metadata else 'FAIL'}")
    print(f"Hash Validation: {hash_validation_pass_rate:.1%} (target: 100%) {'PASS' if check_hash else 'FAIL'}")
    print(f"p95 Latency: {percentiles['p95']:.1f}ms (target: < 2000ms) {'PASS' if check_latency else 'FAIL'}")
    print()

    # Collect issues
    if not check_precision:
        issues.append(f"Only {query_success_rate:.1%} of queries achieved precision@5 >= 0.80 (target: >= 80%)")
    if not check_mrr:
        issues.append(f"MRR {mrr:.3f} below target 0.70")
    if not check_metadata:
        issues.append(f"Metadata completeness {metadata_completeness_rate:.1%} below 100%")
    if not check_hash:
        issues.append(f"Hash validation pass rate {hash_validation_pass_rate:.1%} below 100%")
    if not check_latency:
        issues.append(f"p95 latency {percentiles['p95']:.1f}ms exceeds 2000ms target")

    # Overall result
    overall_pass = all([check_precision, check_mrr, check_metadata, check_hash, check_latency])

    print(f"Overall Result: {'PASS' if overall_pass else 'FAIL'}")
    print()

    if issues:
        print("Issues Identified:")
        for i, issue in enumerate(issues, 1):
            print(f"  {i}. {issue}")
        print()

    # Create ValidationReport
    report = ValidationReport(
        timestamp=datetime.now().isoformat(),
        total_queries=len(test_cases),
        avg_precision_at_5=avg_precision_at_5,
        mrr=mrr,
        avg_latency_ms=avg_latency,
        p95_latency_ms=percentiles['p95'],
        p99_latency_ms=percentiles['p99'],
        metadata_completeness_rate=metadata_completeness_rate,
        hash_validation_pass_rate=hash_validation_pass_rate,
        test_cases=test_cases,
        summary=f"{'PASS' if overall_pass else 'FAIL'}: {queries_meeting_threshold}/{len(test_cases)} queries ({query_success_rate:.1%}) achieved precision@5 >= 0.80. MRR {mrr:.3f}.",
        issues=issues
    )

    # Save JSON report if requested (US7 - Polish)
    if args.output:
        import json
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump(report.to_dict(), f, indent=2, ensure_ascii=False)

        print(f"Validation report saved to: {output_path}")
        print()

    print("=" * 70)
    print(f"Validation Complete - Total Queries: {len(test_cases)}")
    print("=" * 70)

    return 0 if overall_pass else 1


if __name__ == "__main__":
    sys.exit(main())
