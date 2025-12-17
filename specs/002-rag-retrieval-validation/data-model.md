# Data Model: RAG Retrieval and Pipeline Validation

**Feature**: 002-rag-retrieval-validation
**Date**: 2025-12-16
**Purpose**: Define entities, relationships, and validation rules for the validation suite

---

## Entities

### 1. Query

**Description**: Represents a test search query to be executed against the RAG pipeline.

**Fields**:
- `text` (str): The natural language query text (e.g., "How do ROS 2 nodes communicate?")
- `top_k` (int, default=5): Number of results to retrieve
- `query_type` (str): One of "specific", "broad", "paraphrase", "edge"

**Validation Rules**:
- `text` must be non-null (can be empty string for edge case testing)
- `top_k` must be positive integer (1-100)
- `query_type` must be one of the allowed values

**Relationships**:
- One Query → Many SearchResults (via Qdrant query)

**Example**:
```python
{
  "text": "What is a URDF joint?",
  "top_k": 5,
  "query_type": "specific"
}
```

---

### 2. SearchResult

**Description**: A single chunk retrieved from Qdrant in response to a query.

**Fields**:
- `similarity_score` (float): Cosine similarity score (0.0-1.0)
- `content` (str): The chunk text content
- `url` (str): Source page URL
- `title` (str): Source page title
- `chunk_index` (int): Position of this chunk within the source page (0-indexed)
- `timestamp` (str, ISO 8601): When this chunk was ingested
- `content_hash` (str, SHA256 hex): Hash of the content for integrity validation

**Validation Rules**:
- `similarity_score` must be between 0.0 and 1.0
- All string fields must be non-null
- `chunk_index` must be non-negative integer
- `timestamp` must be valid ISO 8601 format
- `content_hash` must be 64-character hexadecimal string (SHA256)

**Relationships**:
- Many SearchResults ← One Query
- One SearchResult → One source page (via URL)

**Example**:
```python
{
  "similarity_score": 0.89,
  "content": "ROS 2 nodes communicate using a publish-subscribe pattern with topics...",
  "url": "https://physical-ai-humanoid-robotics-textb-ivory.vercel.app/docs/module-1-ros2/02-nodes-topics-services",
  "title": "Chapter 2: Nodes, Topics, and Services | Physical AI & Humanoid Robotics",
  "chunk_index": 0,
  "timestamp": "2025-12-15T10:00:00Z",
  "content_hash": "abc123def456..." # SHA256 hex
}
```

---

### 3. TestCase

**Description**: Complete test scenario including query, results, and evaluation.

**Fields**:
- `query` (Query): The test query
- `expected_criteria` (dict): Expected outcome criteria (e.g., {"min_score": 0.7, "expected_topic": "ROS nodes"})
- `actual_results` (List[SearchResult]): Retrieved results from Qdrant
- `relevance_labels` (List[int]): Manual relevance judgments for each result (0=Not Relevant, 1=Relevant, 2=Highly Relevant)
- `precision_at_k` (float): Computed precision@K for this query
- `rank_of_best` (Optional[int]): Position of best result (for MRR calculation), None if no relevant results

**Validation Rules**:
- `relevance_labels` length must match `len(actual_results)`
- Each label must be 0, 1, or 2
- `precision_at_k` must be between 0.0 and 1.0
- `rank_of_best` must be 1-indexed (1 = first position) or None

**Relationships**:
- One TestCase contains one Query
- One TestCase contains many SearchResults

**Example**:
```python
{
  "query": {"text": "How do ROS 2 nodes communicate?", "top_k": 5, "query_type": "specific"},
  "expected_criteria": {"min_score": 0.7, "expected_topic": "ROS 2 topics, pub/sub"},
  "actual_results": [
    # List of 5 SearchResult objects
  ],
  "relevance_labels": [2, 2, 1, 1, 0],  # Highly Relevant, Highly Relevant, Relevant, Relevant, Not Relevant
  "precision_at_k": 0.80,  # 4/5 relevant (counting labels >= 1)
  "rank_of_best": 1  # Best result is first (labels == 2 considered "best")
}
```

---

### 4. ValidationReport

**Description**: Aggregated results across all test cases with pass/fail determination.

**Fields**:
- `timestamp` (str, ISO 8601): When validation was run
- `total_queries` (int): Number of queries tested
- `avg_precision_at_5` (float): Average precision@5 across all queries
- `mrr` (float): Mean Reciprocal Rank across all queries
- `avg_latency_ms` (float): Average end-to-end query latency
- `p95_latency_ms` (float): 95th percentile latency
- `p99_latency_ms` (float): 99th percentile latency
- `metadata_completeness_rate` (float): % of results with all 6 metadata fields non-null
- `hash_validation_pass_rate` (float): % of results where recomputed hash matches stored hash
- `test_cases` (List[TestCase]): Full details for each query
- `summary` (str): Human-readable pass/fail summary
- `issues` (List[str]): List of identified problems or warnings

**Validation Rules**:
- All percentage/rate fields must be between 0.0 and 1.0
- Latency fields must be non-negative
- `total_queries` must match `len(test_cases)`
- `summary` should indicate "PASS" or "FAIL" based on success criteria

**Relationships**:
- One ValidationReport contains many TestCases

**Pass/Fail Criteria** (from spec SC-002, SC-003):
- **PASS** if:
  - At least 80% of queries achieve precision@5 ≥ 0.80
  - MRR ≥ 0.70
  - Metadata completeness = 100%
  - Hash validation pass rate = 100%
  - p95 latency < 2000ms
- **FAIL** otherwise

**Example**:
```python
{
  "timestamp": "2025-12-16T15:30:00Z",
  "total_queries": 15,
  "avg_precision_at_5": 0.82,
  "mrr": 0.74,
  "avg_latency_ms": 450.3,
  "p95_latency_ms": 1200.0,
  "p99_latency_ms": 1800.0,
  "metadata_completeness_rate": 1.0,
  "hash_validation_pass_rate": 1.0,
  "test_cases": [
    # List of TestCase objects
  ],
  "summary": "PASS: 12/15 queries (80%) achieved precision@5 >= 0.80. MRR 0.74 exceeds target 0.70.",
  "issues": [
    "Query 'quantum computing' returned no relevant results (expected - out of domain)",
    "p99 latency 1800ms within acceptable range but approaching limit"
  ]
}
```

---

## Entity Relationships Diagram

```
ValidationReport
├── test_cases: List[TestCase]
│   └── TestCase
│       ├── query: Query
│       ├── actual_results: List[SearchResult]
│       │   └── SearchResult
│       │       ├── similarity_score
│       │       ├── content
│       │       ├── url (→ source page)
│       │       ├── title
│       │       ├── chunk_index
│       │       ├── timestamp
│       │       └── content_hash
│       ├── relevance_labels: List[int]
│       ├── precision_at_k: float
│       └── rank_of_best: Optional[int]
├── metrics (aggregated)
└── summary
```

---

## Metric Computation Formulas

### Precision@K
```python
def compute_precision_at_k(relevance_labels, k):
    """
    Precision@K = (Number of relevant results in top-K) / K

    Args:
        relevance_labels: List[int] where label >= 1 means relevant
        k: Cutoff (e.g., 5 for precision@5)

    Returns:
        float between 0.0 and 1.0
    """
    relevant_count = sum(1 for label in relevance_labels[:k] if label >= 1)
    return relevant_count / k
```

### Mean Reciprocal Rank (MRR)
```python
def compute_mrr(test_cases):
    """
    MRR = Average of (1 / rank of first relevant result) across all queries

    Args:
        test_cases: List[TestCase] where each has rank_of_best

    Returns:
        float between 0.0 and 1.0
    """
    reciprocal_ranks = []
    for case in test_cases:
        if case.rank_of_best is not None:
            reciprocal_ranks.append(1.0 / case.rank_of_best)
        else:
            reciprocal_ranks.append(0.0)  # No relevant result found

    return sum(reciprocal_ranks) / len(test_cases)
```

### Metadata Completeness Rate
```python
def compute_metadata_completeness(results):
    """
    % of results where all 6 metadata fields are non-null

    Required fields: url, title, chunk_index, content, timestamp, content_hash

    Returns:
        float between 0.0 and 1.0
    """
    complete_count = 0
    for result in results:
        if all([
            result.url,
            result.title,
            result.chunk_index is not None,
            result.content,
            result.timestamp,
            result.content_hash
        ]):
            complete_count += 1

    return complete_count / len(results) if results else 1.0
```

### Hash Validation Pass Rate
```python
import hashlib

def compute_hash_validation_rate(results):
    """
    % of results where recomputed hash matches stored hash

    Returns:
        float between 0.0 and 1.0
    """
    pass_count = 0
    for result in results:
        computed_hash = hashlib.sha256(result.content.encode('utf-8')).hexdigest()
        if computed_hash == result.content_hash:
            pass_count += 1

    return pass_count / len(results) if results else 1.0
```

---

## Implementation Notes

1. **Python Dataclasses**: Use `@dataclass` for type safety and automatic `__init__` generation
2. **JSON Serialization**: Implement `to_dict()` methods for easy JSON export
3. **Validation**: Use Pydantic or manual validation for field constraints
4. **Timestamps**: Use `datetime.now().isoformat()` for ISO 8601 format
5. **Hashing**: Use `hashlib.sha256(text.encode('utf-8')).hexdigest()` for consistency with ingestion pipeline

---

## File Storage

Validation reports will be saved to:
```
backend/validation_results/report_YYYYMMDD_HHMMSS.json
```

Filename format: `report_20251216_153045.json` (timestamp when validation started)
