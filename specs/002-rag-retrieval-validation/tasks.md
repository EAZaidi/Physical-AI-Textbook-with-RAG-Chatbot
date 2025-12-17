# Tasks: RAG Retrieval and Pipeline Validation

**Input**: Design documents from `/specs/002-rag-retrieval-validation/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/validation-report-schema.json

**Tests**: Tests are NOT requested in the feature specification. This is a manual validation tool, not an automated test suite.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md, this is a backend validation script:
- Main script: `backend/validate_rag.py`
- Output directory: `backend/validation_results/`
- Environment: `backend/.env` (reused from ingestion pipeline)
- Documentation: `backend/README.md`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and validation script structure

- [X] T001 Create validation script structure in backend/validate_rag.py with argument parsing (--output, --phase flags)
- [X] T002 [P] Create output directory backend/validation_results/ for timestamped reports
- [X] T003 [P] Verify environment variables (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY) loaded from backend/.env

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Initialize Cohere client connection using cohere.Client with API key from .env in backend/validate_rag.py
- [X] T005 Initialize Qdrant client connection using QdrantClient with URL and API key from .env in backend/validate_rag.py
- [X] T006 Verify Qdrant collection 'rag_embedding' exists and retrieve collection info (vector count) in backend/validate_rag.py
- [X] T007 Implement Query dataclass with fields: text, top_k, query_type in backend/validate_rag.py
- [X] T008 [P] Implement SearchResult dataclass with fields: similarity_score, content, url, title, chunk_index, timestamp, content_hash in backend/validate_rag.py
- [X] T009 [P] Implement TestCase dataclass with fields: query, expected_criteria, actual_results, relevance_labels, precision_at_k, rank_of_best in backend/validate_rag.py
- [X] T010 [P] Implement ValidationReport dataclass with fields per schema and to_dict() method for JSON serialization in backend/validate_rag.py
- [X] T011 Define test query list with 15 queries covering diverse topics (ROS 2, URDF, Gazebo, Isaac Sim, VLA, edge cases) in backend/validate_rag.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Basic Semantic Search Validation (Priority: P1) 🎯 MVP

**Goal**: Perform semantic searches against stored embeddings and verify that returned results are relevant to queries

**Independent Test**: Issue a known query (e.g., "How do ROS 2 nodes communicate?"), retrieve top-5 results, manually inspect that returned chunks are semantically relevant. Delivers immediate validation that embeddings and search work.

### Implementation for User Story 1

- [X] T012 [US1] Implement query embedding generation using Cohere embed-english-v3.0 with input_type='search_query' in backend/validate_rag.py
- [X] T013 [US1] Implement Qdrant semantic search using query_points() method with query vector and limit=5 in backend/validate_rag.py
- [X] T014 [US1] Parse Qdrant search results and extract SearchResult objects with all 7 fields (score, content, metadata) in backend/validate_rag.py
- [X] T015 [US1] Implement console output for each query showing: query text, query type, top-5 results with scores in backend/validate_rag.py
- [X] T016 [US1] Implement manual relevance labeling prompt (ask user to rate each result 0/1/2) for each query in backend/validate_rag.py
- [X] T017 [US1] Store relevance labels in TestCase and display summary (how many relevant results found) in backend/validate_rag.py
- [X] T018 [US1] Add error handling for Qdrant connection failures and Cohere API errors in backend/validate_rag.py
- [X] T019 [US1] Add logging to stdout for query execution progress (e.g., "[1/15] Query: ...") in backend/validate_rag.py

**Checkpoint**: At this point, User Story 1 should be fully functional - developer can run validation and manually verify search relevance

---

## Phase 4: User Story 2 - Metadata Integrity Verification (Priority: P2)

**Goal**: Verify that retrieved chunks include complete and accurate metadata (source URL, title, chunk index, content hash, timestamp)

**Independent Test**: Retrieve results from a known query, programmatically validate that each result's payload contains all 6 required fields with non-null, correctly formatted values. Can test without US1's manual labeling.

### Implementation for User Story 2

- [X] T020 [US2] Implement metadata completeness validation checking all 6 required fields are non-null in backend/validate_rag.py
- [X] T021 [US2] Implement content hash validation: recompute SHA256 hash from content field and compare to stored content_hash in backend/validate_rag.py
- [X] T022 [US2] Implement timestamp format validation (ISO 8601) in backend/validate_rag.py
- [X] T023 [US2] Implement URL format validation (valid URI) in backend/validate_rag.py
- [X] T024 [US2] Compute metadata completeness rate across all results (% with all fields non-null) in backend/validate_rag.py
- [X] T025 [US2] Compute hash validation pass rate across all results (% where recomputed hash matches) in backend/validate_rag.py
- [X] T026 [US2] Add console output showing metadata validation results per query in backend/validate_rag.py
- [X] T027 [US2] Add issues list to ValidationReport for failed metadata checks (e.g., "Result X has null URL") in backend/validate_rag.py

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - can validate search AND metadata

---

## Phase 5: User Story 3 - Relevance Quality Assessment (Priority: P3)

**Goal**: Systematically test search quality across diverse query types and measure relevance metrics (precision@K, MRR)

**Independent Test**: Run benchmark of 15 queries with manual relevance labels, compute precision@5 and MRR, compare against target thresholds (precision@5 ≥ 0.80, MRR ≥ 0.70). Can test metric computation independent of US1/US2.

### Implementation for User Story 3

- [X] T028 [US3] Implement precision@K calculation: (number of relevant results in top-K) / K where label >= 1 in backend/validate_rag.py
- [X] T029 [US3] Implement rank_of_best detection: find position of first result with label == 2 (Highly Relevant) in backend/validate_rag.py
- [X] T030 [US3] Implement Mean Reciprocal Rank (MRR) calculation: average of (1 / rank_of_best) across all queries in backend/validate_rag.py
- [X] T031 [US3] Compute average precision@5 across all queries in backend/validate_rag.py
- [X] T032 [US3] Identify queries that fail to meet precision@5 >= 0.80 threshold and add to issues list in backend/validate_rag.py
- [X] T033 [US3] Add metrics summary to console output showing avg precision@5, MRR with target comparisons in backend/validate_rag.py
- [X] T034 [US3] Implement pass/fail determination: PASS if 80% of queries meet precision@5 >= 0.80 AND MRR >= 0.70 in backend/validate_rag.py
- [X] T035 [US3] Add query success rate calculation (% of queries meeting precision threshold) to ValidationReport in backend/validate_rag.py

**Checkpoint**: All core validation metrics (precision@K, MRR) should now be computed and displayed

---

## Phase 6: User Story 4 - End-to-End Pipeline Stress Testing (Priority: P4)

**Goal**: Test retrieval system under realistic load conditions (concurrent queries, large batches) and edge cases (malformed queries, empty results)

**Independent Test**: Simulate concurrent queries using threading, measure response times, verify no errors. Test edge cases with deliberately malformed inputs. Can test stress/edge handling independent of other stories.

### Implementation for User Story 4

- [X] T036 [US4] Implement end-to-end latency measurement (query embedding + Qdrant search) using time.time() in backend/validate_rag.py
- [X] T037 [US4] Store latency for each query in milliseconds in backend/validate_rag.py
- [X] T038 [US4] Compute latency percentiles (p50, p95, p99) using numpy.percentile() in backend/validate_rag.py
- [X] T039 [US4] Add average latency and percentiles to ValidationReport metrics in backend/validate_rag.py
- [X] T040 [US4] Implement edge case handling for empty query strings (return empty results, no errors) in backend/validate_rag.py
- [X] T041 [US4] Implement edge case handling for nonsensical queries (e.g., "asdfghjkl") expecting low confidence results in backend/validate_rag.py
- [X] T042 [US4] Implement edge case handling for out-of-domain queries (e.g., "quantum computing") expecting no relevant results in backend/validate_rag.py
- [X] T043 [US4] Add latency performance checks: flag if p95 >= 2000ms in issues list in backend/validate_rag.py
- [X] T044 [US4] Add console output showing latency summary (avg, p95, p99) with target comparisons in backend/validate_rag.py

**Checkpoint**: All user stories should now be independently functional - full validation suite with metrics, metadata, and stress tests

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Reporting, documentation, and final integration

- [X] T045 [P] Implement JSON report generation with timestamp in filename (report_YYYYMMDD_HHMMSS.json) in backend/validate_rag.py
- [X] T046 [P] Implement ValidationReport JSON serialization following contracts/validation-report-schema.json in backend/validate_rag.py
- [X] T047 Implement --output flag to save JSON report to specified file path in backend/validate_rag.py
- [X] T048 Implement --phase flag to run specific validation phases (P1, P2, P3, P4, all) in backend/validate_rag.py
- [X] T049 Implement final console summary showing overall PASS/FAIL with checkmarks/X marks in backend/validate_rag.py
- [X] T050 Add validation report file path to console output when saved in backend/validate_rag.py
- [X] T051 [P] Update backend/README.md with validation script usage instructions from quickstart.md
- [X] T052 [P] Add troubleshooting section to backend/README.md for common validation errors
- [X] T053 Test complete validation workflow: run script, label results, review JSON report in backend/validate_rag.py
- [X] T054 Verify all success criteria from spec.md are measurable in the validation report

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User stories can then proceed in priority order (P1 → P2 → P3 → P4)
  - Each story builds on previous but should remain independently testable
- **Polish (Phase 7)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Uses search results from US1 but validates different aspect (metadata vs relevance)
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Computes metrics on relevance labels from US1
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - Adds latency measurement and edge case handling independent of metrics

### Within Each User Story

- US1: Query embedding → Search → Result parsing → Console output → Relevance labeling
- US2: Metadata validation → Hash validation → Completeness rate → Console output
- US3: Precision@K → MRR → Success rate → Pass/fail determination
- US4: Latency measurement → Percentiles → Edge cases → Performance checks

### Parallel Opportunities

**Setup Phase (Phase 1)**:
- T002 and T003 can run in parallel with T001

**Foundational Phase (Phase 2)**:
- T004 and T005 can run in parallel (different clients)
- T007, T008, T009, T010 can run in parallel (different dataclasses)

**User Story 1 (Phase 3)**:
- Limited parallelization (sequential query execution pipeline)

**User Story 2 (Phase 4)**:
- T020, T021, T022, T023 can run in parallel (different validation functions)

**User Story 3 (Phase 5)**:
- T028, T029 can run in parallel (different metric functions)

**User Story 4 (Phase 6)**:
- T040, T041, T042 can run in parallel (different edge case handlers)

**Polish Phase (Phase 7)**:
- T045, T046 can run in parallel with T051, T052 (code vs docs)

---

## Parallel Example: Foundational Phase

```bash
# Launch all dataclass definitions together:
Task: "Implement SearchResult dataclass with fields: similarity_score, content, url, title, chunk_index, timestamp, content_hash in backend/validate_rag.py"
Task: "Implement TestCase dataclass with fields: query, expected_criteria, actual_results, relevance_labels, precision_at_k, rank_of_best in backend/validate_rag.py"
Task: "Implement ValidationReport dataclass with fields per schema and to_dict() method for JSON serialization in backend/validate_rag.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Basic Semantic Search Validation)
4. **STOP and VALIDATE**: Run validation script with 3-5 test queries, manually verify relevance
5. If P1 works, pipeline is validated for basic search - can proceed to agent integration

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test semantic search independently → **MVP READY** (can validate basic retrieval works)
3. Add User Story 2 → Test metadata integrity independently → **METADATA VALIDATION READY**
4. Add User Story 3 → Compute metrics independently → **QUANTITATIVE VALIDATION READY**
5. Add User Story 4 → Test stress/edge cases independently → **PRODUCTION-READY VALIDATION**
6. Add Polish → JSON reports and documentation → **COMPLETE VALIDATION SUITE**

### Sequential Implementation (Recommended)

Single developer workflow:

1. Complete Setup + Foundational together (T001-T011)
2. Implement User Story 1 (T012-T019) → Test with sample queries
3. Implement User Story 2 (T020-T027) → Test metadata checks
4. Implement User Story 3 (T028-T035) → Compute and verify metrics
5. Implement User Story 4 (T036-T044) → Test latency and edge cases
6. Implement Polish (T045-T054) → Generate reports and finalize docs

---

## Task Summary

- **Total Tasks**: 54
- **Setup Phase**: 3 tasks
- **Foundational Phase**: 8 tasks (CRITICAL - blocks everything)
- **User Story 1 (P1 - MVP)**: 8 tasks
- **User Story 2 (P2)**: 8 tasks
- **User Story 3 (P3)**: 8 tasks
- **User Story 4 (P4)**: 9 tasks
- **Polish Phase**: 10 tasks

### Tasks per User Story

- **US1 (Basic Semantic Search)**: 8 tasks - Core search validation
- **US2 (Metadata Integrity)**: 8 tasks - Data quality validation
- **US3 (Relevance Quality)**: 8 tasks - Quantitative metrics
- **US4 (Stress Testing)**: 9 tasks - Performance and edge cases

### Parallel Opportunities Identified

- **Setup Phase**: 2 parallel groups
- **Foundational Phase**: 2 parallel groups (clients, dataclasses)
- **User Story 2**: 4 parallel validation functions
- **User Story 3**: 2 parallel metric functions
- **User Story 4**: 3 parallel edge case handlers
- **Polish Phase**: 2 parallel groups (code vs docs)

### Suggested MVP Scope

**Minimum Viable Product**: Phase 1 (Setup) + Phase 2 (Foundational) + Phase 3 (User Story 1)
- **Total**: 19 tasks (T001-T019)
- **Delivers**: Basic semantic search validation with manual relevance checking
- **Value**: Confirms RAG pipeline works before agent integration

---

## Notes

- All tasks are in a single file (backend/validate_rag.py) - no parallel file editing conflicts
- [P] tasks are marked where functions can be implemented independently (different dataclasses, different validation logic)
- [Story] label maps task to specific user story for traceability
- Each user story delivers independent value and can be tested separately
- Tests are NOT included (manual validation tool, not automated test suite per spec)
- Stop at any checkpoint to validate independently
- Commit after each user story phase completion
