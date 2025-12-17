# Implementation Plan: RAG Retrieval and Pipeline Validation

**Branch**: `002-rag-retrieval-validation` | **Date**: 2025-12-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-rag-retrieval-validation/spec.md`
**User Requirements**: Initialize Qdrant client and collection access, run semantic similarity queries against stored vectors, validate relevance and metadata mapping, measure basic retrieval accuracy and latency.

## Summary

Build a validation suite that systematically tests the RAG pipeline created in 001-rag-ingestion-pipeline by performing semantic searches against the Qdrant vector database, verifying result relevance and metadata integrity, and measuring retrieval quality metrics (precision@K, MRR) and performance (latency). Implementation will be a single Python validation script with test queries covering diverse topics from the Physical AI & Humanoid Robotics textbook, enabling developers to confirm pipeline reliability before AI agent integration.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**:
- `cohere` (Cohere Python SDK for query embeddings - reuse from ingestion pipeline)
- `qdrant-client` (Qdrant Python client for semantic search)
- `python-dotenv` (environment variable management)
- `hashlib` (built-in, for content hash validation)

**Storage**: Read-only access to existing Qdrant collection `rag_embedding` (no new storage)
**Testing**: Manual validation via script execution, no automated unit tests required for MVP
**Target Platform**: Linux/macOS/Windows (development), same as ingestion pipeline
**Project Type**: Single-file validation script (backend/validate_rag.py or similar)
**Performance Goals**:
- Complete validation suite in <5 minutes for 10-15 test queries
- p95 query latency <2 seconds
- Precision@5 ≥ 0.80 for 80% of queries

**Constraints**:
- Read-only operations (no writes to Qdrant)
- Reuse existing Cohere API key and Qdrant credentials from .env
- Manual relevance judgments (no automated ground truth dataset)
- Validation must work with 45-200 chunks (current corpus size)

**Scale/Scope**:
- 10-15 test queries covering diverse textbook topics (ROS 2, URDF, simulation, control, VLA)
- 4 validation scenarios (P1-P4 from spec)
- Single validation report output (console + optional JSON file)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Source Accuracy & Verifiability
✅ **PASS**: Validation suite tests retrieval quality against known textbook content. All test queries will reference actual textbook topics. No new technical claims introduced.

### Educational Clarity & Student Success
✅ **PASS**: This is development tooling for pipeline validation, not student-facing tutorial code. However, the validation script will include clear console output and explanatory comments for future maintainers.

### Reproducibility & Environment Consistency
✅ **PASS**: Validation script reuses the same environment as 001-rag-ingestion-pipeline (Python 3.11+, uv package manager, same .env file). No new dependencies beyond what's already installed.

### Spec-Driven Content Development
✅ **PASS**: Following full spec → plan → tasks workflow per Spec-Kit Plus methodology.

### RAG Chatbot Fidelity
✅ **PASS**: This validation ensures zero-hallucination by verifying that retrieved content is relevant and traceable to source URLs. Metadata validation confirms source attribution capability.

### Modular Architecture & Progressive Complexity
✅ **PASS**: Validation suite implements 4 progressive user stories (P1: basic search, P2: metadata, P3: metrics, P4: stress tests). MVP is P1 only - validate semantic search relevance manually.

### Production-Ready Deployment Standards
✅ **PASS**:
- Environment variables for API keys (reuse .env from ingestion pipeline)
- Error handling for Qdrant/Cohere connection failures
- Logging to stdout for observability
- Validation script will be documented in README

**Complexity Justification**: Single-file validation script (<300 LOC estimated) is simpler than multi-module architecture. Acceptable for this scope. Manual relevance judgments are standard practice for search quality evaluation when no labeled dataset exists.

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-retrieval-validation/
├── spec.md              # Feature requirements (completed)
├── plan.md              # This file (in progress)
├── research.md          # Phase 0: Best practices for search quality evaluation
├── data-model.md        # Phase 1: ValidationReport, TestCase, SearchResult entities
├── quickstart.md        # Phase 1: Setup and usage instructions
├── contracts/           # Phase 1: ValidationReport JSON schema
│   └── validation-report-schema.json
└── tasks.md             # Phase 2: Implementation tasks (/sp.tasks command)
```

### Source Code (repository root)

```text
backend/
├── main.py                  # Existing ingestion pipeline
├── validate_rag.py          # NEW: Validation script (this feature)
├── test_queries.json        # NEW: Test query definitions (optional)
├── .env                     # Existing (reused for Qdrant/Cohere credentials)
├── pyproject.toml           # Existing (no new dependencies needed)
└── README.md                # Updated with validation instructions

validation_results/          # NEW: Optional output directory
└── report_YYYYMMDD_HHMMSS.json  # Timestamped validation reports
```

**Structure Decision**: Single validation script in `backend/` directory alongside the ingestion pipeline. This keeps related code together and reuses the same Python environment. Test queries will be embedded in the script initially (can be externalized to JSON later if needed). Validation reports will be printed to console and optionally saved to a timestamped JSON file.

## Complexity Tracking

> **No violations** - all constitution principles pass without justification required.

## Phase 0: Research

### Research Topics

1. **Search Quality Metrics Best Practices**
   - **Decision**: Use Precision@K and Mean Reciprocal Rank (MRR)
   - **Rationale**:
     - Precision@K measures relevance at cutoff K (standard for top-K retrieval)
     - MRR measures ranking quality (rewards placing best answer higher)
     - Both are simple to compute and interpret
     - Widely used in information retrieval research
   - **Alternatives considered**:
     - Normalized Discounted Cumulative Gain (NDCG): More complex, requires graded relevance (not binary)
     - Recall@K: Requires knowing total relevant documents (infeasible with manual labeling)
     - F1@K: Less common for top-K retrieval tasks

2. **Manual Relevance Judgment Strategies**
   - **Decision**: Binary relevance (relevant/not relevant) with 3-point scale for edge cases
   - **Rationale**:
     - Binary judgments are faster and have higher inter-annotator agreement
     - 3-point scale (Highly Relevant=2, Relevant=1, Not Relevant=0) allows nuance for borderline cases
     - Single annotator (developer) acceptable for MVP validation
   - **Alternatives considered**:
     - 5-point Likert scale: Too granular, slow for manual labeling
     - Multiple annotators: Expensive, overkill for internal validation
     - Automated metrics (BM25, semantic similarity): Don't capture true relevance without ground truth

3. **Semantic Search Query Construction**
   - **Decision**: Mix of specific technical queries and broad conceptual queries
   - **Rationale**:
     - Specific queries (e.g., "What is a URDF mesh?") test precision
     - Broad queries (e.g., "How do humanoid robots work?") test diversity
     - Paraphrase variants test semantic understanding beyond keyword matching
     - Cover all major textbook modules (ROS 2, URDF, Gazebo, Isaac Sim, VLA)
   - **Alternatives considered**:
     - Only specific queries: Doesn't test real user queries (which are often vague)
     - Only broad queries: Doesn't test ability to find specific information
     - Auto-generated queries from content: Risk of biased/unrealistic queries

4. **Qdrant Query API Best Practices**
   - **Decision**: Use `query_points` with query vector and limit parameter
   - **Rationale**:
     - `query_points` returns scored results sorted by similarity
     - Supports batch queries for efficiency
     - Returns full payload (metadata) with each result
   - **Alternatives considered**:
     - `search` method: Deprecated in newer Qdrant versions
     - `scroll` method: For pagination, not scoring-based retrieval
     - Direct vector search: Lower-level, less convenient

5. **Latency Measurement Strategy**
   - **Decision**: Measure end-to-end latency (query embedding + Qdrant search)
   - **Rationale**:
     - Realistic latency from user perspective
     - Cohere embedding time is significant (50-500ms)
     - Qdrant search is typically fast (<50ms for small collections)
     - Report p50, p95, p99 percentiles for distribution understanding
   - **Alternatives considered**:
     - Measure Qdrant-only latency: Doesn't reflect real usage (embedding always required)
     - Average latency only: Hides outliers and tail latency issues
     - Median latency only: Doesn't capture worst-case performance

6. **Content Hash Validation Approach**
   - **Decision**: Recompute SHA256 hash from retrieved content, compare to stored hash
   - **Rationale**:
     - Detects data corruption or encoding issues in Qdrant storage
     - SHA256 is cryptographically secure and fast
     - Byte-for-byte comparison ensures no silent data changes
   - **Alternatives considered**:
     - Skip hash validation: Misses silent data corruption
     - Use checksums (CRC32): Less secure, similar performance
     - Validate metadata only: Doesn't catch content corruption

7. **Validation Report Format**
   - **Decision**: Structured JSON with metrics summary + per-query details
   - **Rationale**:
     - Machine-readable for potential future automation
     - Human-readable when pretty-printed
     - Can be timestamped and archived for trend analysis
     - Console output shows summary, full JSON saved to file
   - **Alternatives considered**:
     - Plain text only: Hard to parse programmatically
     - CSV format: Awkward for nested data (queries, results)
     - Database storage: Overkill for MVP, adds complexity

## Phase 1: Design

### Data Model

**Entities** (defined in data-model.md):

1. **Query**
   - Fields: text (str), top_k (int, default=5), query_type (str: "specific" | "broad" | "paraphrase")
   - Purpose: Represents a test search query
   - Relationships: One Query → Many SearchResults

2. **SearchResult**
   - Fields: similarity_score (float), content (str), url (str), title (str), chunk_index (int), timestamp (str), content_hash (str)
   - Purpose: Single result returned by Qdrant
   - Relationships: Many SearchResults ← One Query

3. **TestCase**
   - Fields: query (Query), expected_criteria (dict), actual_results (List[SearchResult]), relevance_labels (List[int]), precision_at_k (float), rank_of_best (Optional[int])
   - Purpose: Complete test scenario with ground truth and evaluation
   - Relationships: One TestCase contains one Query and many SearchResults

4. **ValidationReport**
   - Fields: timestamp (str), total_queries (int), avg_precision_at_5 (float), mrr (float), avg_latency_ms (float), p95_latency_ms (float), p99_latency_ms (float), metadata_completeness_rate (float), hash_validation_pass_rate (float), test_cases (List[TestCase]), summary (str), issues (List[str])
   - Purpose: Aggregated validation results across all test cases
   - Relationships: One ValidationReport contains many TestCases

### API Contracts

**ValidationReport JSON Schema** (contracts/validation-report-schema.json):

```json
{
  "timestamp": "2025-12-16T15:30:00Z",
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
      "query": {"text": "How do ROS 2 nodes communicate?", "top_k": 5, "query_type": "specific"},
      "actual_results": [
        {
          "similarity_score": 0.89,
          "content": "...",
          "url": "https://...",
          "title": "ROS 2 Nodes and Topics",
          "chunk_index": 0,
          "timestamp": "2025-12-15T10:00:00Z",
          "content_hash": "abc123..."
        }
      ],
      "relevance_labels": [2, 2, 1, 1, 0],
      "precision_at_5": 0.80,
      "rank_of_best": 1
    }
  ],
  "summary": "PASS: 12/15 queries (80%) achieved precision@5 >= 0.80. MRR 0.74 exceeds target 0.70.",
  "issues": [
    "Query 'quantum computing' returned no relevant results (expected - out of domain)",
    "p99 latency 1800ms slightly exceeds 2000ms target"
  ]
}
```

### Quickstart (quickstart.md preview)

**Prerequisites**:
- 001-rag-ingestion-pipeline complete, Qdrant collection populated
- Python 3.11+, uv installed
- Same .env file with COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY

**Usage**:
```bash
cd backend
uv run python validate_rag.py

# Optional: Save detailed report
uv run python validate_rag.py --output validation_results/report.json

# Optional: Run specific validation phase (P1, P2, P3, P4)
uv run python validate_rag.py --phase P1
```

**Expected Output**:
```
=== RAG Pipeline Validation ===
Testing 15 queries...

[1/15] Query: "How do ROS 2 nodes communicate?"
  Top 5 results retrieved in 420ms
  Relevance: [Highly Relevant, Highly Relevant, Relevant, Relevant, Not Relevant]
  Precision@5: 0.80 ✓

...

=== VALIDATION SUMMARY ===
PASS: 12/15 queries (80%) achieved precision@5 >= 0.80
MRR: 0.74 (target: >= 0.70) ✓
Avg Latency: 450ms (target: < 500ms) ✓
p95 Latency: 1200ms (target: < 2000ms) ✓
Metadata Completeness: 100% ✓
Hash Validation: 100% ✓

Overall: PASS ✅
```

## Implementation Phases

### Phase 2: Task Breakdown

Will be generated by `/sp.tasks` command.

**Expected task categories**:
1. **Setup Tasks**: Initialize script structure, load environment variables, connect to Qdrant
2. **Query Execution Tasks** (P1): Implement query embedding, Qdrant search, result parsing
3. **Metadata Validation Tasks** (P2): Validate payload completeness, recompute hashes
4. **Metrics Computation Tasks** (P3): Implement precision@K, MRR calculation, latency measurement
5. **Stress Testing Tasks** (P4): Concurrent query execution, edge case handling
6. **Reporting Tasks**: Generate console output, save JSON report
7. **Documentation Tasks**: Update README, create usage examples

### Test Queries (Preview)

Will be embedded in `validate_rag.py`:

```python
TEST_QUERIES = [
    # Specific technical queries (P1)
    {"text": "How do ROS 2 nodes communicate?", "type": "specific", "expected_topic": "nodes, topics, pub/sub"},
    {"text": "What is a URDF joint?", "type": "specific", "expected_topic": "URDF, robot modeling"},
    {"text": "How do I create a Gazebo world file?", "type": "specific", "expected_topic": "Gazebo simulation"},

    # Broad conceptual queries (P1)
    {"text": "How do humanoid robots work?", "type": "broad", "expected_topic": "multiple modules"},
    {"text": "What is perception in robotics?", "type": "broad", "expected_topic": "cameras, sensors, vision"},

    # Paraphrase variants (P1)
    {"text": "ROS 2 message passing", "type": "paraphrase", "variant_of": "How do ROS 2 nodes communicate?"},
    {"text": "Robot messaging systems", "type": "paraphrase", "variant_of": "How do ROS 2 nodes communicate?"},

    # Module-specific queries (P1)
    {"text": "Unity Robotics simulation setup", "type": "specific", "module": 2},
    {"text": "NVIDIA Isaac Sim getting started", "type": "specific", "module": 2},
    {"text": "Vision-Language-Action models", "type": "specific", "module": 6},

    # Edge cases (P4)
    {"text": "", "type": "edge", "expected": "empty result or error"},
    {"text": "asdfghjkl", "type": "edge", "expected": "low confidence results"},
    {"text": "quantum computing algorithms", "type": "edge", "expected": "out of domain"},
]
```

## ADR Recommendations

Based on this plan, consider creating ADRs for:

1. **ADR-001: Manual Relevance Judgments for MVP**
   - **Decision**: Use single-annotator binary relevance labels instead of automated metrics
   - **Rationale**: No pre-existing labeled dataset, automated metrics don't capture true relevance
   - **Tradeoffs**: Manual labeling is slow but accurate for small query set (15 queries)

2. **ADR-002: Precision@K and MRR as Primary Metrics**
   - **Decision**: Prioritize precision@K and MRR over NDCG and recall
   - **Rationale**: Simple to compute and interpret, standard in IR research
   - **Tradeoffs**: Less nuanced than NDCG, but sufficient for validation purposes

3. **ADR-003: Single-File Validation Script**
   - **Decision**: Implement as standalone script rather than integrated test suite
   - **Rationale**: Simpler for developers to run, no test framework dependencies
   - **Tradeoffs**: Not integrated with CI/CD, but acceptable for manual validation tool

## Next Steps

1. Run `/sp.tasks` to generate detailed implementation task breakdown
2. Implement P1 (Basic Semantic Search Validation) first as MVP
3. Manually label relevance for initial 10-15 test queries
4. Run validation suite and iterate on query selection based on results
5. Optional: Implement P2-P4 for comprehensive validation before agent integration
