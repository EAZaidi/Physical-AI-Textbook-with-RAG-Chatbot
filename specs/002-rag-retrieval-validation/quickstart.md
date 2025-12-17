# Quickstart: RAG Retrieval and Pipeline Validation

**Feature**: 002-rag-retrieval-validation
**Purpose**: Quick guide to running the RAG validation suite

---

## Prerequisites

1. **Completed RAG Ingestion**: Feature 001-rag-ingestion-pipeline must be complete with Qdrant collection populated
2. **Python Environment**: Python 3.11+ with uv package manager installed
3. **API Credentials**: Same `.env` file from ingestion pipeline with:
   - `COHERE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
4. **Dependencies**: All dependencies from ingestion pipeline (cohere, qdrant-client, python-dotenv)

---

## Quick Start

### 1. Navigate to Backend Directory

```bash
cd backend
```

### 2. Run Validation Script

**Basic Usage** (MVP - P1 only):
```bash
uv run python validate_rag.py
```

**Save Detailed Report**:
```bash
uv run python validate_rag.py --output validation_results/report.json
```

**Run Specific Phase**:
```bash
# P1: Basic semantic search validation only
uv run python validate_rag.py --phase P1

# P2: Metadata integrity validation
uv run python validate_rag.py --phase P2

# P3: Relevance quality metrics
uv run python validate_rag.py --phase P3

# P4: Stress testing and edge cases
uv run python validate_rag.py --phase P4

# All phases
uv run python validate_rag.py --phase all
```

---

## Expected Output

### Console Output (Summary)

```
=== RAG Pipeline Validation ===
Feature: 002-rag-retrieval-validation
Qdrant URL: https://0f6a2c90-9b53-42e8-aa19-1058db62f2e5.us-east4-0.gcp.cloud.qdrant.io:6333
Collection: rag_embedding
Total vectors: 45

Testing 15 queries...

[1/15] Query: "How do ROS 2 nodes communicate?"
  Query Type: specific
  Top 5 results retrieved in 420ms
  Relevance Judgments: [Highly Relevant, Highly Relevant, Relevant, Relevant, Not Relevant]
  Precision@5: 0.80 ✓
  Rank of Best Result: 1

[2/15] Query: "What is a URDF joint?"
  Query Type: specific
  Top 5 results retrieved in 380ms
  Relevance Judgments: [Highly Relevant, Relevant, Relevant, Not Relevant, Not Relevant]
  Precision@5: 0.60 ✗ (below threshold)
  Rank of Best Result: 1

...

[15/15] Query: "quantum computing algorithms"
  Query Type: edge
  Top 5 results retrieved in 450ms
  Relevance Judgments: [Not Relevant, Not Relevant, Not Relevant, Not Relevant, Not Relevant]
  Precision@5: 0.00 (expected - out of domain)
  Rank of Best Result: None

=== VALIDATION SUMMARY ===

Metrics:
  - Avg Precision@5: 0.82 (target: >= 0.80 for 80% of queries) ✓
  - MRR: 0.74 (target: >= 0.70) ✓
  - Avg Latency: 450ms (typical case) ✓
  - p95 Latency: 1200ms (target: < 2000ms) ✓
  - p99 Latency: 1800ms ✓
  - Metadata Completeness: 100% ✓
  - Hash Validation: 100% ✓

Query Success Rate:
  - 12/15 queries (80%) achieved precision@5 >= 0.80 ✓

Issues Identified:
  1. Query "What is a URDF joint?" precision@5 = 0.60 (below 0.80 threshold)
  2. Query "quantum computing" returned no relevant results (expected - out of domain)
  3. Query "asdfghjkl" returned low confidence results (expected - nonsense query)

Overall: PASS ✅

Full report saved to: validation_results/report_20251216_153045.json
```

### JSON Report (Detailed)

Saved to `backend/validation_results/report_YYYYMMDD_HHMMSS.json`:

```json
{
  "timestamp": "2025-12-16T15:30:45Z",
  "total_queries": 15,
  "metrics": {
    "avg_precision_at_5": 0.82,
    "mrr": 0.74,
    "avg_latency_ms": 450.3,
    "p95_latency_ms": 1200.0,
    "p99_latency_ms": 1800.0,
    "metadata_completeness_rate": 1.0,
    "hash_validation_pass_rate": 1.0
  },
  "test_cases": [
    {
      "query": {
        "text": "How do ROS 2 nodes communicate?",
        "top_k": 5,
        "query_type": "specific"
      },
      "expected_criteria": {
        "min_score": 0.7,
        "expected_topic": "ROS 2 topics, pub/sub"
      },
      "actual_results": [
        {
          "similarity_score": 0.89,
          "content": "ROS 2 nodes communicate using a publish-subscribe pattern...",
          "url": "https://physical-ai-humanoid-robotics-textb-ivory.vercel.app/docs/module-1-ros2/02-nodes-topics-services",
          "title": "Chapter 2: Nodes, Topics, and Services",
          "chunk_index": 0,
          "timestamp": "2025-12-15T10:00:00Z",
          "content_hash": "abc123..."
        }
      ],
      "relevance_labels": [2, 2, 1, 1, 0],
      "precision_at_k": 0.80,
      "rank_of_best": 1
    }
  ],
  "summary": "PASS: 12/15 queries (80%) achieved precision@5 >= 0.80. MRR 0.74 exceeds target 0.70.",
  "issues": [
    "Query 'What is a URDF joint?' precision@5 = 0.60 (below threshold)",
    "Query 'quantum computing' returned no relevant results (expected - out of domain)",
    "p99 latency 1800ms within acceptable range but approaching limit"
  ]
}
```

---

## Interpreting Results

### Success Criteria (from spec)

The validation **PASSES** if:
- ✓ At least 80% of queries achieve precision@5 ≥ 0.80 (SC-002)
- ✓ MRR ≥ 0.70 (SC-003)
- ✓ All retrieved results include complete metadata (100%, SC-004)
- ✓ Content hash validation passes for 100% of results (SC-005)
- ✓ p95 latency < 2000ms (SC-006)

### Metrics Explained

- **Precision@5**: What percentage of the top 5 results are relevant? (0.80 = 4/5 relevant)
- **MRR (Mean Reciprocal Rank)**: How highly ranked is the best result on average? (0.70 = typically in top 3)
- **Latency Percentiles**:
  - **p95**: 95% of queries complete faster than this
  - **p99**: 99% of queries complete faster than this (worst-case excluding outliers)

### Common Issues

1. **Low Precision for Specific Queries**:
   - **Cause**: Query too specific, not enough relevant content in corpus
   - **Action**: Review chunking strategy (may need smaller chunks) or add more content

2. **Low MRR**:
   - **Cause**: Best results not ranking high enough
   - **Action**: Review embedding model choice or consider re-chunking for better context

3. **Metadata Incomplete**:
   - **Cause**: Ingestion pipeline issue
   - **Action**: Re-run 001-rag-ingestion-pipeline

4. **Hash Validation Failures**:
   - **Cause**: Data corruption in Qdrant or encoding issues
   - **Action**: Re-ingest affected pages

5. **High Latency (p95 > 2000ms)**:
   - **Cause**: Cohere API slowness or network issues
   - **Action**: Retry validation, check network connectivity

---

## Manual Relevance Labeling

For P3 (Relevance Quality Assessment), you'll need to manually label result relevance:

1. Run validation script (it will prompt for labels)
2. For each result, enter:
   - `2` = Highly Relevant (directly answers query)
   - `1` = Relevant (related information)
   - `0` = Not Relevant (off-topic)

**Tips**:
- Label based on whether the content would help answer the query
- Consider context: "Highly Relevant" should directly address the question
- Be consistent across queries
- Document uncertain labels for review

---

## Troubleshooting

### Error: "Qdrant collection not found"

**Cause**: 001-rag-ingestion-pipeline not run or collection name mismatch

**Solution**:
```bash
# Run ingestion pipeline first
cd backend
uv run python main.py

# Verify collection exists
uv run python -c "
from qdrant_client import QdrantClient
from dotenv import load_dotenv
import os
load_dotenv()
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))
print(f'Collections: {[c.name for c in client.get_collections().collections]}')
"
```

### Error: "Cohere API key invalid"

**Cause**: Missing or incorrect COHERE_API_KEY in .env

**Solution**:
```bash
# Check .env file
cat backend/.env | grep COHERE_API_KEY

# Update if needed
nano backend/.env
```

### Error: "No results returned for query"

**Cause**: Query embedding or Qdrant connection issue

**Solution**:
1. Check network connectivity to Qdrant Cloud
2. Verify query text is not empty
3. Test Qdrant connection manually (see above)

---

## Next Steps

After validation passes:
1. ✅ Validation complete - RAG pipeline ready for agent integration
2. 🔄 If issues found: Iterate on chunking strategy or re-ingest content
3. 📊 Optional: Run validation periodically to detect quality regressions
4. 🚀 Ready to integrate with AI agent (e.g., ChatGPT, Claude, custom chatbot)

---

## Additional Options

### Custom Test Queries

Edit `backend/validate_rag.py` and modify the `TEST_QUERIES` list:

```python
TEST_QUERIES = [
    {"text": "Your custom query here", "top_k": 5, "query_type": "specific"},
    # ... more queries
]
```

### Save All Validation Reports

Create a validation history:

```bash
mkdir -p backend/validation_results
uv run python validate_rag.py --output validation_results/report_$(date +%Y%m%d_%H%M%S).json
```

### Compare Validation Runs

```bash
# Run validation before changes
uv run python validate_rag.py --output validation_results/before.json

# Make changes (e.g., update chunking)
# ...

# Run validation after changes
uv run python validate_rag.py --output validation_results/after.json

# Compare metrics
python -c "
import json
before = json.load(open('validation_results/before.json'))
after = json.load(open('validation_results/after.json'))
print(f'Precision@5: {before[\"metrics\"][\"avg_precision_at_5\"]:.3f} -> {after[\"metrics\"][\"avg_precision_at_5\"]:.3f}')
print(f'MRR: {before[\"metrics\"][\"mrr\"]:.3f} -> {after[\"metrics\"][\"mrr\"]:.3f}')
"
```

---

## Support

For issues or questions:
- **Documentation**: See [spec.md](./spec.md) and [plan.md](./plan.md)
- **GitHub Issues**: [physical-ai-humanoid-book/issues](https://github.com/yourusername/physical-ai-humanoid-book/issues)
