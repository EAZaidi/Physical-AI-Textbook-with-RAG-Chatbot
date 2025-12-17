# Research: RAG Retrieval and Pipeline Validation

**Feature**: 002-rag-retrieval-validation
**Date**: 2025-12-16
**Purpose**: Document technology decisions and best practices for building a RAG pipeline validation suite

---

## 1. Search Quality Metrics Best Practices

**Research Question**: Which metrics should we use to evaluate semantic search quality?

**Decision**: Use **Precision@K** and **Mean Reciprocal Rank (MRR)** as primary metrics.

**Rationale**:
- **Precision@K**: Measures what percentage of top-K results are relevant. Simple to compute and interpret. Industry standard for top-K retrieval evaluation.
- **MRR**: Measures ranking quality by rewarding systems that place the best answer higher. Commonly used in Q&A and search evaluation.
- Both metrics are:
  - Easy to explain to stakeholders
  - Simple to compute (no complex weighting)
  - Widely used in information retrieval research
  - Work with binary relevance judgments (relevant/not relevant)

**Alternatives Considered**:
1. **Normalized Discounted Cumulative Gain (NDCG)**:
   - Pros: Handles graded relevance (0-5 scale), rewards better ranking
   - Cons: More complex to compute and explain, requires graded relevance labels (slower manual labeling)
   - Decision: Defer to post-MVP if graded relevance is needed

2. **Recall@K**:
   - Pros: Measures coverage (what % of all relevant docs are in top-K)
   - Cons: Requires knowing total relevant documents (infeasible with manual labeling - would need to label entire corpus)
   - Decision: Not practical for manual evaluation

3. **F1@K**:
   - Pros: Balances precision and recall
   - Cons: Less common for top-K retrieval, requires recall calculation
   - Decision: Precision@K alone is sufficient for validation

**Formula Reference**:
- Precision@K = (Number of relevant results in top-K) / K
- MRR = Average of (1 / rank of first relevant result) across all queries

**Target Values** (from spec):
- Precision@5: ≥ 0.80 (4/5 results relevant on average)
- MRR: ≥ 0.70 (best answer typically in top 3 positions)

---

## 2. Manual Relevance Judgment Strategies

**Research Question**: How should we label relevance when no ground truth dataset exists?

**Decision**: Use **3-point relevance scale** (Highly Relevant=2, Relevant=1, Not Relevant=0) with single annotator.

**Rationale**:
- **Binary vs Graded**: Pure binary (relevant/not) loses nuance for borderline cases. 3-point scale adds minimal complexity while capturing important distinctions.
- **Single Annotator**: For MVP internal validation, developer can label queries. Multiple annotators expensive and unnecessary for 15 queries.
- **3-Point Scale**:
  - **Highly Relevant (2)**: Directly answers query or provides key information
  - **Relevant (1)**: Contains related information but not directly on-topic
  - **Not Relevant (0)**: Unrelated or off-topic

**Alternatives Considered**:
1. **5-Point Likert Scale** (Very Relevant, Relevant, Neutral, Not Relevant, Very Not Relevant):
   - Pros: More granular feedback
   - Cons: Too many categories lead to low inter-annotator agreement, slower labeling
   - Decision: 3-point sufficient for validation

2. **Multiple Annotators**:
   - Pros: Higher reliability, can measure inter-annotator agreement (IAA)
   - Cons: Expensive (3x time cost), overkill for internal validation tool
   - Decision: Single annotator acceptable for MVP

3. **Automated Metrics** (BM25 similarity, semantic similarity scores):
   - Pros: No manual labeling needed
   - Cons: Don't capture true relevance (keyword matching ≠ semantic relevance), no ground truth
   - Decision: Manual labels required for validation

**Labeling Guidelines**:
- Label top-5 results for each query (75 labels total for 15 queries)
- Estimated time: 10-15 minutes (30-60 seconds per query)
- Document uncertain labels for review

---

## 3. Semantic Search Query Construction

**Research Question**: What types of queries should we test to ensure comprehensive validation?

**Decision**: Mix of **specific technical queries**, **broad conceptual queries**, and **paraphrase variants** covering all major textbook modules.

**Rationale**:
- **Specific Queries** (e.g., "What is a URDF joint?"):
  - Test precision on narrow topics
  - Mimic lookup/reference use cases
  - Should return highly focused results
- **Broad Queries** (e.g., "How do humanoid robots work?"):
  - Test diversity and coverage
  - Mimic exploratory learning use cases
  - Should return varied relevant chunks (not duplicates)
- **Paraphrase Variants** (e.g., "ROS messaging" vs "ROS topics"):
  - Test semantic understanding beyond keyword matching
  - Verify embedding model captures synonyms
  - Should return similar top results

**Alternatives Considered**:
1. **Only Specific Queries**:
   - Pros: Easy to evaluate (clear right/wrong)
   - Cons: Doesn't test real user behavior (users often ask vague questions)
   - Decision: Include both specific and broad

2. **Only Broad Queries**:
   - Pros: Tests diversity
   - Cons: Doesn't test ability to find specific information
   - Decision: Include both

3. **Auto-Generated Queries** (from textbook headings):
   - Pros: Scalable, no manual query writing
   - Cons: Risk of biased/unrealistic queries (e.g., "Chapter 2.3: ROS 2 Nodes" not natural)
   - Decision: Manually curate realistic queries

**Query Distribution** (for 15 queries):
- 5 specific technical (ROS 2, URDF, Gazebo, Isaac Sim, VLA)
- 3 broad conceptual (perception, control, humanoid overview)
- 3 paraphrase variants (test 1-2 concepts with multiple phrasings)
- 3 module-specific (ensure coverage across all 7 modules)
- 1-2 edge cases (empty query, out-of-domain, nonsense)

**Module Coverage**:
- Module 1: ROS 2 (2-3 queries)
- Module 2: Simulation (2 queries)
- Module 3: Perception (1 query)
- Module 4-5: Planning/Manipulation (1 query)
- Module 6: VLA models (1 query)
- Module 7: Capstone (0 queries - too high-level for current corpus)

---

## 4. Qdrant Query API Best Practices

**Research Question**: What Qdrant API method should we use for semantic search?

**Decision**: Use **`query_points`** method with query vector and limit parameter.

**Rationale**:
- **`query_points`**:
  - Returns scored results sorted by similarity (descending)
  - Supports top-K limiting via `limit` parameter
  - Returns full payload (all metadata fields)
  - Efficient for single queries
- **Query Construction**:
  1. Generate query embedding via Cohere (`co.embed(texts=[query], model='embed-english-v3.0', input_type='search_query')`)
  2. Call `qdrant_client.query_points(collection_name='rag_embedding', query=query_vector, limit=K)`
  3. Extract results with scores and payloads

**Alternatives Considered**:
1. **`search` method**:
   - Pros: Similar API
   - Cons: Deprecated in newer Qdrant versions (query_points recommended)
   - Decision: Use query_points for future compatibility

2. **`scroll` method**:
   - Pros: Pagination support for large result sets
   - Cons: Not score-based retrieval (iterates all points), slower
   - Decision: Not suitable for semantic search

3. **Direct vector search** (lower-level API):
   - Pros: Maximum control
   - Cons: More code, less convenient
   - Decision: query_points provides sufficient control

**API Example**:
```python
from qdrant_client import QdrantClient
import cohere

co = cohere.Client(api_key)
qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

# Generate query embedding
query_embedding = co.embed(
    texts=["How do ROS 2 nodes communicate?"],
    model="embed-english-v3.0",
    input_type="search_query"  # Note: search_query for queries, search_document for docs
).embeddings[0]

# Search Qdrant
results = qdrant_client.query_points(
    collection_name="rag_embedding",
    query=query_embedding,
    limit=5
).points

# Extract results
for result in results:
    print(f"Score: {result.score:.4f}")
    print(f"Content: {result.payload['content'][:100]}...")
    print(f"URL: {result.payload['url']}")
```

**Note**: Use `input_type='search_query'` for queries (not `search_document`). This optimizes Cohere embeddings for query-document matching.

---

## 5. Latency Measurement Strategy

**Research Question**: How should we measure query latency to reflect real-world usage?

**Decision**: Measure **end-to-end latency** (query embedding + Qdrant search) and report **p50, p95, p99 percentiles**.

**Rationale**:
- **End-to-End Latency**:
  - Reflects actual user experience (both steps required)
  - Cohere embedding time is significant (50-500ms depending on API load)
  - Qdrant search is fast (<50ms for small collections like ours)
  - Sum of both gives realistic total latency
- **Percentile Reporting**:
  - **p50 (median)**: Typical case latency
  - **p95**: Worst case for 95% of queries (outlier threshold)
  - **p99**: Maximum tail latency
  - Percentiles reveal distribution better than just average

**Alternatives Considered**:
1. **Qdrant-Only Latency**:
   - Pros: Isolates search performance
   - Cons: Doesn't reflect real usage (embedding always needed)
   - Decision: Include both for observability

2. **Average Latency Only**:
   - Pros: Simple to understand
   - Cons: Hides outliers (one 5s query masked by nine 500ms queries)
   - Decision: Report percentiles instead

3. **Median Only**:
   - Pros: Robust to outliers
   - Cons: Doesn't capture tail latency (p95/p99 important for UX)
   - Decision: Report all three (p50, p95, p99)

**Measurement Implementation**:
```python
import time

latencies = []
for query in test_queries:
    start_time = time.time()

    # Step 1: Generate embedding (measure this separately too)
    embedding_start = time.time()
    query_embedding = co.embed(texts=[query], model="embed-english-v3.0", input_type="search_query").embeddings[0]
    embedding_time = time.time() - embedding_start

    # Step 2: Search Qdrant
    search_start = time.time()
    results = qdrant_client.query_points(collection_name="rag_embedding", query=query_embedding, limit=5)
    search_time = time.time() - search_start

    end_to_end_time = time.time() - start_time
    latencies.append(end_to_end_time)

# Compute percentiles
import numpy as np
p50 = np.percentile(latencies, 50)
p95 = np.percentile(latencies, 95)
p99 = np.percentile(latencies, 99)
```

**Target Latencies** (from spec):
- p95 < 2000ms (2 seconds) for standard queries (5-20 words)
- Average < 500ms (implied from "typical case")

---

## 6. Content Hash Validation Approach

**Research Question**: How can we detect data corruption in Qdrant storage?

**Decision**: **Recompute SHA256 hash** from retrieved content field and compare to stored `content_hash`.

**Rationale**:
- **SHA256 Hash**:
  - Cryptographically secure (collision-resistant)
  - Fast to compute (modern CPUs: ~500MB/s)
  - Already used in ingestion pipeline (consistency)
  - Detects any single-byte change in content
- **Validation Process**:
  1. Retrieve result from Qdrant
  2. Extract `content` and `content_hash` from payload
  3. Recompute: `computed_hash = hashlib.sha256(content.encode('utf-8')).hexdigest()`
  4. Compare: `assert computed_hash == content_hash`
  5. Report: % of results with matching hashes

**Alternatives Considered**:
1. **Skip Hash Validation**:
   - Pros: Faster validation
   - Cons: Silent data corruption undetected (e.g., Qdrant encoding issues, network transmission errors)
   - Decision: Validate hashes for data integrity

2. **Use Checksums (CRC32)**:
   - Pros: Faster to compute
   - Cons: Not collision-resistant, less secure
   - Decision: SHA256 preferred (already in pipeline, minimal performance difference)

3. **Validate Metadata Only** (URLs, titles):
   - Pros: Simpler
   - Cons: Doesn't catch content corruption (the most important field)
   - Decision: Validate content hashes

**Validation Code**:
```python
import hashlib

def validate_content_hash(result):
    content = result.payload['content']
    stored_hash = result.payload['content_hash']

    # Recompute hash
    computed_hash = hashlib.sha256(content.encode('utf-8')).hexdigest()

    # Compare
    return computed_hash == stored_hash

# Run validation
hash_validation_results = [validate_content_hash(result) for result in all_results]
pass_rate = sum(hash_validation_results) / len(hash_validation_results)
print(f"Hash Validation Pass Rate: {pass_rate:.1%}")
```

**Success Criterion** (from spec):
- SC-005: 100% hash validation pass rate (no data corruption)

---

## 7. Validation Report Format

**Research Question**: How should validation results be presented to developers?

**Decision**: **Structured JSON** with metrics summary + per-query details, printed to console and optionally saved to file.

**Rationale**:
- **JSON Format**:
  - Machine-readable (can be parsed by CI/CD tools later)
  - Human-readable when pretty-printed
  - Standard format for API responses and data exchange
  - Supports nested structures (queries, results, metrics)
- **Dual Output**:
  - **Console**: Summary view for quick validation (pass/fail, key metrics)
  - **JSON File**: Full details for deep analysis and archiving
  - Timestamped files enable trend analysis over time

**Alternatives Considered**:
1. **Plain Text Only**:
   - Pros: Simple, readable
   - Cons: Hard to parse programmatically, no structured data for automation
   - Decision: JSON preferred for future extensibility

2. **CSV Format**:
   - Pros: Easy to import into Excel/Sheets
   - Cons: Awkward for nested data (queries → results), loses hierarchical structure
   - Decision: JSON better for complex data

3. **Database Storage** (PostgreSQL, SQLite):
   - Pros: Queryable, enables historical analysis
   - Cons: Overkill for MVP, adds dependency and complexity
   - Decision: Defer to post-MVP if trend analysis needed

**Report Structure** (see plan.md for full schema):
```json
{
  "timestamp": "2025-12-16T15:30:00Z",
  "total_queries": 15,
  "metrics": {
    "avg_precision_at_5": 0.82,
    "mrr": 0.74,
    "avg_latency_ms": 450.3,
    "p95_latency_ms": 1200.0,
    "metadata_completeness_rate": 1.0,
    "hash_validation_pass_rate": 1.0
  },
  "test_cases": [...],
  "summary": "PASS: 12/15 queries (80%) achieved precision@5 >= 0.80",
  "issues": [...]
}
```

**Console Output Example**:
```
=== VALIDATION SUMMARY ===
PASS: 12/15 queries (80%) achieved precision@5 >= 0.80 ✓
MRR: 0.74 (target: >= 0.70) ✓
Avg Latency: 450ms ✓
p95 Latency: 1200ms ✓
Metadata Completeness: 100% ✓
Hash Validation: 100% ✓

Overall: PASS ✅
```

---

## Summary

All 7 research topics have been resolved with clear decisions and rationales. No further clarifications needed. Ready to proceed to Phase 1 (Design).
