---
id: 002
title: Create RAG validation plan
stage: plan
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5
feature: 002-rag-retrieval-validation
branch: 002-rag-retrieval-validation
user: Essa Zaidi
command: /sp.plan
labels: ["plan", "validation", "rag", "retrieval", "architecture", "testing"]
links:
  spec: specs/002-rag-retrieval-validation/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-rag-retrieval-validation/plan.md
  - specs/002-rag-retrieval-validation/research.md
  - specs/002-rag-retrieval-validation/data-model.md
  - specs/002-rag-retrieval-validation/contracts/validation-report-schema.json
  - specs/002-rag-retrieval-validation/quickstart.md
tests:
  - Constitution Check: 7/7 principles PASS
  - Phase 0 Research: 7 decisions documented with rationales
  - Phase 1 Design: 4 entities, JSON schema, quickstart guide
---

## Prompt

/sp.plan - Initialize Qdrant client and collection access, run semantic similarity queries against stored vectors, validate relevance and metadata mapping, measure basic retrieval accuracy and latency.

## Response snapshot

Created comprehensive implementation plan for RAG Retrieval and Pipeline Validation feature with full architecture, research, data model, and contracts.

**Plan Structure**:
- **Technical Context**: Python 3.11+, reuse dependencies from 001-rag-ingestion-pipeline (cohere, qdrant-client, python-dotenv), single validation script (<300 LOC)
- **Constitution Check**: 7/7 principles PASS (no violations)
- **Project Structure**: backend/validate_rag.py + validation_results/ directory for reports
- **Complexity Tracking**: No justification needed - all gates pass

**Phase 0: Research** (7 decisions documented):
1. **Search Quality Metrics**: Precision@K + MRR (vs NDCG, Recall@K, F1@K)
2. **Manual Relevance Judgments**: 3-point scale (0-2) with single annotator (vs 5-point, multiple annotators, automated metrics)
3. **Query Construction**: Mix of specific, broad, paraphrase queries covering all modules (vs only specific, only broad, auto-generated)
4. **Qdrant API**: Use `query_points` method (vs deprecated `search`, `scroll`, direct vector search)
5. **Latency Measurement**: End-to-end (embedding + search) with p50/p95/p99 percentiles (vs Qdrant-only, average-only, median-only)
6. **Content Hash Validation**: Recompute SHA256, compare to stored hash (vs skip validation, CRC32, metadata-only)
7. **Report Format**: Structured JSON + console summary (vs plain text, CSV, database)

**Phase 1: Design**:
- **Data Model** (4 entities):
  - **Query**: text, top_k, query_type (specific | broad | paraphrase | edge)
  - **SearchResult**: similarity_score, content, url, title, chunk_index, timestamp, content_hash (6 metadata fields)
  - **TestCase**: query, expected_criteria, actual_results, relevance_labels (0-2), precision_at_k, rank_of_best
  - **ValidationReport**: timestamp, metrics (7 values), test_cases, summary, issues
- **API Contracts**: JSON schema with full validation rules (JSON Schema Draft 7)
- **Quickstart**: Prerequisites, usage examples, output interpretation, troubleshooting

**Implementation Strategy**:
- **MVP (P1)**: Basic semantic search validation with manual relevance judgments (10-15 queries)
- **P2**: Metadata integrity (all 6 fields complete + hash validation)
- **P3**: Quantitative metrics (precision@K, MRR computation)
- **P4**: Stress testing (concurrent queries, edge cases)

**Test Queries** (15 planned):
- 5 specific technical (ROS 2, URDF, Gazebo, Isaac Sim, VLA)
- 3 broad conceptual (perception, control, humanoid overview)
- 3 paraphrase variants (semantic understanding)
- 3 module-specific (coverage across 7 modules)
- 1-2 edge cases (empty, nonsense, out-of-domain)

**Success Criteria Mapping**:
- SC-001: Validation completes in <5 min (estimated: 2-3 min for 15 queries)
- SC-002: 80% queries @ precision@5 ≥ 0.80 (12/15 queries pass)
- SC-003: MRR ≥ 0.70 (best answer typically top 3)
- SC-004-005: 100% metadata complete + hash valid
- SC-006: p95 latency <2000ms
- SC-007-008: Concurrent queries + edge case handling
- SC-009-010: Diverse query types + actionable insights

**ADR Recommendations** (3 suggested):
1. Manual Relevance Judgments for MVP (no automated metrics)
2. Precision@K and MRR as Primary Metrics (vs NDCG, recall)
3. Single-File Validation Script (vs test suite integration)

## Outcome

- ✅ Impact: Complete implementation plan ready for task breakdown and implementation
- 🧪 Tests: Constitution Check 7/7 PASS, all research decisions documented, data model complete with formulas
- 📁 Files: Created plan.md, research.md (7 decisions), data-model.md (4 entities + formulas), validation-report-schema.json, quickstart.md
- 🔁 Next prompts: Ready for `/sp.tasks` to generate detailed task breakdown for P1-P4 implementation
- 🧠 Reflection: Plan successfully balances comprehensive validation (P1-P4) with practical MVP approach (P1 manual validation first). Research phase resolved all technical decisions with clear rationales and alternatives considered. Data model provides strong typing and validation rules for implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None - all planning phases completed successfully
- Graders run and results (PASS/FAIL): Constitution Check (PASS - 7/7), Research Completeness (PASS - 7 decisions documented), Data Model Quality (PASS - 4 entities with validation rules), Contract Specification (PASS - JSON schema complete), Quickstart Quality (PASS - prerequisites, usage, troubleshooting)
- Prompt variant (if applicable): Validation/testing feature planning (builds on 001-rag-ingestion-pipeline infrastructure)
- Next experiment (smallest change to try): Proceed to `/sp.tasks` to break down P1-P4 into actionable implementation tasks with acceptance criteria, then implement P1 (MVP) with 10-15 test queries and manual labeling
