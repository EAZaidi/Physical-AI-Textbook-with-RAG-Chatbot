---
description: "Task breakdown for RAG Agent API implementation"
---

# Tasks: RAG Agent API

**Input**: Design documents from `/specs/003-rag-agent-api/`
**Prerequisites**: plan.md (complete), spec.md (complete), research.md (complete), data-model.md (complete), contracts/api-schema.yaml (complete)

**Tests**: Tests are NOT explicitly requested in the spec, so test tasks are EXCLUDED. Focus on implementation only.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

Project structure follows backend/agent_api/ pattern per plan.md:
- Backend API: `backend/agent_api/`
- Tests: `backend/tests/`
- Configuration: `backend/.env`, `backend/pyproject.toml`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure (0.5-1 hour)

- [x] T001 Create backend/agent_api/ directory structure with __init__.py, main.py, agent.py, tools.py, models.py, sessions.py, confidence.py, config.py
- [x] T002 [P] Create backend/agent_api/routers/ directory with __init__.py, chat.py, sessions.py
- [x] T003 [P] Initialize pyproject.toml with dependencies: fastapi>=0.110.0, uvicorn[standard]>=0.30.0, openai>=1.55.0, qdrant-client>=1.12.0, sqlalchemy>=2.0.0, pydantic>=2.0.0, python-dotenv>=1.0.0, psycopg2-binary>=2.9.0
- [x] T004 [P] Create .env.example with all required environment variables per quickstart.md
- [x] T005 [P] Create backend/Dockerfile with multi-stage build (base + runtime) per plan.md
- [x] T006 [P] Create backend/docker-compose.yml with postgres and qdrant services per plan.md

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented (1-2 hours)

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T007 Implement backend/agent_api/config.py with environment variable loading (OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL), validated settings, and constants (MAX_QUERY_LENGTH=1000, MAX_RESPONSE_TOKENS=1000, RATE_LIMIT=100, TOP_K_DEFAULT=5, SIMILARITY_THRESHOLD=0.7)
- [x] T008 [P] Implement backend/agent_api/models.py with Pydantic models: ChatRequest (message, session_id, stream, top_k, similarity_threshold), ChatResponse (response, confidence, sources, session_id, timestamp, confidence_level, should_answer), Source (chunk_text, similarity_score, chapter, section, url, chunk_index)
- [x] T009 [P] Implement backend/agent_api/sessions.py with SQLAlchemySession factory (get_session function), database engine configuration (pool_size=20, max_overflow=40, pool_pre_ping=True), and session cleanup utilities
- [x] T010 Create backend/agent_api/db/ directory with __init__.py and init_db.py script to create conversation_threads table with schema per data-model.md (thread_id UUID PRIMARY KEY, messages JSONB, created_at TIMESTAMPTZ, updated_at TIMESTAMPTZ, topic_summary TEXT, metadata JSONB)
- [x] T011 [P] Create backend/agent_api/prompts/ directory with system.txt containing strict RAG instructions per research.md Layer 1 (ABSOLUTE REQUIREMENTS: MUST use search_textbook only, refuse if average_score < 0.75, no general knowledge, always cite chunks)
- [x] T012 Implement backend/agent_api/main.py FastAPI application with: app initialization, lifespan context for database setup/teardown, health check endpoint (GET /health) with Qdrant/OpenAI/PostgreSQL status checks, CORS configuration, API key authentication middleware, rate limiting middleware (100 req/min per key)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Basic Question Answering (Priority: P1) 🎯 MVP

**Goal**: Enable developers to send questions and receive accurate, grounded answers with source citations (2-3 hours)

**Independent Test**: Send POST to `/chat/run` with "What is a ROS 2 node?" → receive accurate answer with sources and confidence score

### Implementation for User Story 1

- [x] T013 [P] [US1] Implement RetrievalResult model in backend/agent_api/models.py (chunks: list[str], scores: list[float], metadata: list[ChunkMetadata], average_score: float, has_sufficient_context: bool, query_embedding: list[float])
- [x] T014 [P] [US1] Implement ChunkMetadata model in backend/agent_api/models.py (chapter: str, section: str, url: str, chunk_index: int, content_hash: str)
- [x] T015 [US1] Implement search_textbook @function_tool in backend/agent_api/tools.py: async function that generates query embedding (OpenAI text-embedding-3-small), searches Qdrant collection "textbook_chunks" (cosine similarity, score_threshold=0.7), returns RetrievalResult with chunks/scores/metadata/average_score
- [x] T016 [US1] Implement create_rag_agent function in backend/agent_api/agent.py: creates Agent with name="Textbook Assistant", model="gpt-4o-mini", loads system instructions from prompts/system.txt, registers search_textbook tool, accepts SQLAlchemySession parameter
- [x] T017 [US1] Implement POST /chat/run endpoint in backend/agent_api/routers/chat.py: validates ChatRequest, gets/creates agent with session, runs agent synchronously (asyncio.to_thread), extracts sources from response events, calculates basic confidence score (average similarity), returns ChatResponse with response/sources/confidence/session_id
- [x] T018 [US1] Implement extract_sources helper function in backend/agent_api/routers/chat.py: parses agent response events, extracts tool_call results with search_textbook, maps chunks to Source objects with metadata (chapter, section, url, chunk_index, similarity_score)
- [x] T019 [US1] Add error handling to /chat/run endpoint in backend/agent_api/routers/chat.py: catch Qdrant connection errors (HTTP 503), OpenAI API errors (retry with exponential backoff), input validation errors (HTTP 400), return actionable error messages per US4 requirements
- [x] T020 [US1] Add logging to /chat/run endpoint in backend/agent_api/routers/chat.py: log query, retrieved chunks, generated response, confidence score, execution time (meets FR-009)
- [x] T021 [US1] Implement basic refusal logic in backend/agent_api/tools.py: check if average_score < 0.4 OR len(chunks) < 1, return empty RetrievalResult with has_sufficient_context=False (agent will refuse per system instructions)

**Checkpoint**: At this point, User Story 1 should be fully functional - can ask questions, get grounded answers with citations, and agent refuses low-quality retrievals

---

## Phase 4: User Story 2 - Conversation Context Management (Priority: P2)

**Goal**: Enable multi-turn conversations with history persistence (1-2 hours)

**Independent Test**: Send "What is URDF?" followed by "Can you give an example?" to same session_id → agent understands "example" refers to URDF

### Implementation for User Story 2

- [ ] T022 [P] [US2] Implement Message model in backend/agent_api/models.py (role: str, content: str, timestamp: datetime, confidence: Optional[float])
- [ ] T023 [P] [US2] Implement ConversationThread model in backend/agent_api/models.py (thread_id: str, messages: list[Message], created_at: datetime, updated_at: datetime, topic_summary: Optional[str], metadata: dict)
- [ ] T024 [US2] Update get_session function in backend/agent_api/sessions.py: add session persistence to conversation_threads table, create new thread_id if not exists, load existing messages from database if thread_id provided
- [ ] T025 [US2] Implement save_message function in backend/agent_api/sessions.py: append user/assistant messages to conversation_threads.messages JSONB column, update updated_at timestamp, limit to last 50 messages (archive older)
- [ ] T026 [US2] Update /chat/run endpoint in backend/agent_api/routers/chat.py: save user message before agent call, save assistant response after agent call, pass conversation history (last 5 turns = 10 messages) to agent context
- [ ] T027 [US2] Implement GET /session/{session_id}/history endpoint in backend/agent_api/routers/sessions.py: retrieve conversation_threads row by thread_id, return SessionHistoryResponse with messages, created_at, updated_at, topic_summary
- [ ] T028 [US2] Implement SessionHistoryResponse model in backend/agent_api/models.py (thread_id: str, messages: list[Message], created_at: datetime, updated_at: datetime, topic_summary: Optional[str])
- [ ] T029 [US2] Implement DELETE /session/{session_id} endpoint in backend/agent_api/routers/sessions.py: delete conversation_threads row by thread_id, clear agent cache for session_id, return 204 No Content
- [ ] T030 [US2] Implement agent caching in backend/agent_api/agent.py: maintain dict cache {session_id: Agent} to avoid recreation overhead, evict after 1 hour inactive (SESSION_EXPIRY_HOURS=1 from config)
- [ ] T031 [US2] Implement conversation summarization in backend/agent_api/sessions.py: when conversation reaches 10+ exchanges, generate topic_summary using OpenAI completion (max 200 chars), store in conversation_threads.topic_summary column
- [ ] T032 [US2] Add session isolation validation in backend/agent_api/sessions.py: ensure concurrent conversations from different users don't cross-contaminate (separate SQLAlchemySession instances per thread_id)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - can have multi-turn conversations, retrieve history, delete sessions, and context is maintained correctly

---

## Phase 5: User Story 3 - Response Quality Validation (Priority: P3)

**Goal**: Enable confidence-based answer assessment to detect unreliable responses (1-1.5 hours)

**Independent Test**: Send well-covered question → confidence >0.8 + high quality. Send edge case → confidence <0.5 + disclaimer or refusal.

### Implementation for User Story 3

- [x] T033 [P] [US3] Implement ConfidenceMetrics model in backend/agent_api/models.py (average_similarity: float, min_similarity: float, max_similarity: float, num_chunks: int, chunk_diversity: float, confidence_level: str, should_answer: bool)
- [x] T034 [US3] Implement ConfidenceMetrics.calculate classmethod in backend/agent_api/confidence.py: compute multi-metric assessment from RetrievalResult (5 metrics per research.md Decision 5), return confidence level ("high" | "medium" | "low" | "insufficient") and should_answer decision
- [x] T035 [US3] Implement confidence level thresholds in backend/agent_api/confidence.py: HIGH (avg>=0.85, min>=0.75, count>=3), MEDIUM (avg>=0.75, min>=0.65, count>=2), LOW (avg>=0.60, count>=1, REFUSE), INSUFFICIENT (avg<0.60, REFUSE)
- [x] T036 [US3] Implement chunk diversity calculation in backend/agent_api/confidence.py: compute pairwise cosine similarities between chunk embeddings, diversity = 1.0 - mean(pairwise_sims), handle edge case (single chunk: diversity=0.0)
- [x] T037 [US3] Update search_textbook tool in backend/agent_api/tools.py: calculate ConfidenceMetrics from RetrievalResult, include metrics in tool response, enable agent to assess retrieval quality before answering
- [x] T038 [US3] Update /chat/run endpoint in backend/agent_api/routers/chat.py: extract ConfidenceMetrics from agent response, populate ChatResponse.confidence_level and ChatResponse.should_answer fields, include disclaimer for medium confidence ("limited information available")
- [x] T039 [US3] Implement adaptive threshold logic in backend/agent_api/confidence.py: adjust confidence thresholds based on query type (factual vs conceptual), consider chunk diversity in final decision
- [x] T040 [US3] Add confidence logging in backend/agent_api/routers/chat.py: log all 5 confidence metrics (average, min, max, count, diversity), log refusal decisions with reason, enable quality monitoring per FR-009
- [x] T041 [US3] Update system prompt in backend/agent_api/prompts/system.txt: add instructions to refuse when ConfidenceMetrics.should_answer=False, provide explicit disclaimer for medium confidence answers

**Checkpoint**: All user stories (1, 2, 3) should now be independently functional - confidence scoring works, refusal logic triggers correctly, and disclaimers appear for low-quality retrievals

---

## Phase 6: User Story 4 - API Error Handling and Resilience (Priority: P4)

**Goal**: Graceful degradation and actionable error messages for all failure scenarios (1.5-2 hours)

**Independent Test**: Simulate Qdrant offline → HTTP 503 with clear message. Send malformed JSON → HTTP 400 with specific field error. Exceed rate limit → HTTP 429.

### Implementation for User Story 4

- [x] T042 [P] [US4] Implement exponential backoff retry decorator in backend/agent_api/utils/retry.py: use tenacity library with stop_after_attempt(5), wait_exponential(multiplier=1, min=1, max=16), retry on OpenAI rate limit errors
- [x] T043 [P] [US4] Implement CircuitBreaker class in backend/agent_api/utils/circuit_breaker.py: track failure count, open circuit after 5 consecutive failures, timeout 60 seconds, reset on success, raise HTTP 503 when circuit open
- [x] T044 [US4] Wrap OpenAI embedding calls in backend/agent_api/tools.py with exponential backoff retry: apply @retry decorator to openai_client.embeddings.create, handle rate limit errors (429), log retry attempts
- [x] T045 [US4] Wrap Qdrant search calls in backend/agent_api/tools.py with circuit breaker: use CircuitBreaker.call() for qdrant_client.search, catch connection errors, return HTTP 503 with message "Vector search service unavailable, please retry"
- [x] T046 [US4] Implement comprehensive input validation in backend/agent_api/routers/chat.py: validate message length (1-1000 chars) → HTTP 400, validate session_id format (UUID v4) → HTTP 400, validate top_k range (1-10) → HTTP 400, validate similarity_threshold (0.0-1.0) → HTTP 400
- [x] T047 [US4] Implement rate limit tracking in backend/agent_api/main.py middleware: use in-memory token bucket (dev) or Redis (production), track requests per API key, return HTTP 429 with Retry-After header when limit exceeded (100 req/min per FR-008)
- [x] T048 [US4] Implement request size validation in backend/agent_api/main.py middleware: reject requests >10KB with HTTP 413 "Payload Too Large", indicate max allowed size in error message
- [x] T049 [US4] Implement error response models in backend/agent_api/models.py: ErrorResponse (error_code: str, message: str, detail: Optional[str], suggested_action: Optional[str]), define error codes for all scenarios (QDRANT_UNAVAILABLE, OPENAI_RATE_LIMIT, INVALID_INPUT, etc.)
- [x] T050 [US4] Update all exception handlers in backend/agent_api/routers/chat.py: catch specific exceptions (QdrantConnectionError, OpenAIRateLimitError, ValidationError), return ErrorResponse with actionable messages, include suggested_action field ("retry in 60 seconds", "reduce query length", etc.)
- [x] T051 [US4] Implement structured logging in backend/agent_api/utils/logger.py: use Python logging with JSON formatter, log level from LOG_LEVEL env var, include request_id, session_id, error stack traces, log all error responses for debugging
- [x] T052 [US4] Update health check endpoint in backend/agent_api/main.py: add dependency checks (Qdrant, OpenAI, PostgreSQL), measure latency for each service, return detailed status (up/down) with latency_ms, overall status="healthy" only if all services up

**Checkpoint**: All user stories (1-4) complete - error handling is comprehensive, error messages are actionable, resilience strategies work (retry, circuit breaker)

---

## Phase 7: Streaming Support (Additional Feature)

**Purpose**: Enable real-time streaming responses via Server-Sent Events (1-1.5 hours)

- [x] T053 [P] Implement POST /chat/stream endpoint in backend/agent_api/routers/chat.py: validates ChatRequest with stream=true, creates event_generator async function, returns StreamingResponse with media_type="text/event-stream"
- [x] T054 Implement event_generator in backend/agent_api/routers/chat.py: async for loop over agent.run_stream_async(), yield SSE events (data: for raw_response tokens, event: tool_call for retrieval, event: done for completion with usage stats, event: error for failures)
- [x] T055 Add SSE headers to streaming response in backend/agent_api/routers/chat.py: Cache-Control: no-cache, Connection: keep-alive, X-Accel-Buffering: no (disable nginx buffering)
- [x] T056 [P] Update ChatRequest model in backend/agent_api/models.py: add stream: bool field (default=false), update validation to allow stream parameter
- [x] T057 Implement streaming error handling in backend/agent_api/routers/chat.py: wrap event_generator in try-except, yield error events (event: error with detail message), close stream gracefully on exceptions
- [x] T058 Add streaming examples to backend/agent_api/routers/chat.py docstring: curl command with --no-buffer flag, expected SSE event format, event types (raw_response, tool_call, done, error)

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Documentation, deployment, and final validation (1-2 hours)

- [x] T059 [P] Update README.md in backend/ directory: add project description, link to quickstart.md, list features (basic Q&A, conversation context, confidence scoring, error handling, streaming), prerequisites, quick start commands
- [x] T060 [P] Create deployment guide in specs/003-rag-agent-api/deployment.md: Docker image build instructions, Kubernetes manifests (Deployment, Service, Ingress), environment-specific configs (dev/staging/prod), monitoring setup (health check intervals), API key rotation procedures
- [x] T061 Validate quickstart.md instructions in specs/003-rag-agent-api/quickstart.md: run through full setup (uv sync, docker-compose up, uvicorn start), make test requests (health, /chat/run, /chat/stream), verify all curl examples work, ensure 15-minute setup time target is met (SC-010)
- [x] T062 [P] Create API usage examples in backend/examples/ directory: example_sync.py (basic synchronous chat), example_stream.py (streaming chat with SSE parsing), example_conversation.py (multi-turn conversation), example_error_handling.py (handling 503/429 errors)
- [x] T063 [P] Document configuration options in specs/003-rag-agent-api/configuration.md: list all .env variables with descriptions, explain confidence thresholds (CONFIDENCE_THRESHOLD_HIGH, MEDIUM, LOW), document rate limiting (API_RATE_LIMIT), session expiry (SESSION_EXPIRY_HOURS)
- [x] T064 Implement startup validation in backend/agent_api/main.py: check all required environment variables on startup (OPENAI_API_KEY, QDRANT_URL, DATABASE_URL), fail fast with clear error if missing, validate Qdrant collection exists, test database connection
- [x] T065 [P] Create .dockerignore in backend/ directory: exclude .env, __pycache__/, *.pyc, .pytest_cache/, logs/, venv/
- [x] T066 Add final constitution compliance check: verify all 7 principles met (Source Accuracy ✓, Educational Clarity ✓, Reproducibility ✓, Spec-Driven ✓, RAG Fidelity ✓, Modular Architecture ✓, Production Standards ✓)
- [x] T067 Run full system integration test: start all services (docker-compose up), run uvicorn, execute quickstart success checklist (all 9 items), verify SC-001 through SC-010 success criteria, test error scenarios (Qdrant down, rate limit, out-of-domain questions)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup (Phase 1) completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational (Phase 2) completion - MVP deliverable
- **User Story 2 (Phase 4)**: Depends on Foundational (Phase 2) + optionally US1 for integration - Can run in parallel with US3/US4 if staffed
- **User Story 3 (Phase 5)**: Depends on Foundational (Phase 2) + US1 (needs search_textbook tool) - Can run in parallel with US2/US4 if staffed
- **User Story 4 (Phase 6)**: Depends on Foundational (Phase 2) - Can run in parallel with US2/US3 if staffed
- **Streaming Support (Phase 7)**: Depends on US1 completion (needs /chat endpoint structure) - Optional enhancement
- **Polish (Phase 8)**: Depends on all desired user stories being complete (US1-US4 minimum)

### User Story Dependencies

- **User Story 1 (P1) - Basic Q&A**: Can start after Foundational - No dependencies on other stories - **MVP SCOPE**
- **User Story 2 (P2) - Conversation Context**: Can start after Foundational - Integrates with US1 endpoint but independently testable
- **User Story 3 (P3) - Confidence Scoring**: Can start after US1 (extends search_textbook tool) - Independently testable with mock data
- **User Story 4 (P4) - Error Handling**: Can start after Foundational - Applies to all endpoints, independently testable via failure simulation

### Critical Path (Sequential MVP Delivery)

1. **Phase 1 (Setup)** → 2. **Phase 2 (Foundational)** → 3. **Phase 3 (US1 - MVP)** → 4. **Validate MVP** → 5. **Phase 4 (US2)** → 6. **Phase 5 (US3)** → 7. **Phase 6 (US4)** → 8. **Phase 7 (Streaming)** → 9. **Phase 8 (Polish)**

**MVP Milestone**: After Phase 3 (US1), the agent can answer questions with grounded responses and citations. This delivers immediate value and validates core RAG functionality.

### Parallel Opportunities

**Within Setup (Phase 1)**:
- T002 (directory structure), T003 (pyproject.toml), T004 (.env.example), T005 (Dockerfile), T006 (docker-compose.yml) - all [P] tasks can run in parallel

**Within Foundational (Phase 2)**:
- T008 (models.py), T009 (sessions.py), T011 (prompts/system.txt) - all [P] tasks can run in parallel after T007 (config.py)

**Within User Story 1 (Phase 3)**:
- T013 (RetrievalResult model), T014 (ChunkMetadata model) - both [P] can run in parallel

**Across User Stories (if team capacity allows)**:
- After Phase 2 completes, US2, US3, US4 can all start in parallel (US3 needs US1 T015 for search_textbook tool, but can stub it initially)

**Within Phase 8 (Polish)**:
- T059 (README), T060 (deployment guide), T062 (examples), T063 (configuration docs), T065 (.dockerignore) - all [P] can run in parallel

---

## Parallel Example: User Story 1 (Basic Q&A)

After Foundational phase completes, these tasks can run in parallel:

```bash
# Terminal 1: Models
cd backend/agent_api
# Work on T013 (RetrievalResult model) in models.py

# Terminal 2: Models
cd backend/agent_api
# Work on T014 (ChunkMetadata model) in models.py

# Then sequentially:
# T015 (search_textbook tool in tools.py) - requires T013, T014
# T016 (create_rag_agent in agent.py) - requires T015
# T017 (POST /chat/run endpoint in routers/chat.py) - requires T016
# T018 (extract_sources helper in routers/chat.py) - can run parallel with T017
# T019 (error handling) - requires T017
# T020 (logging) - requires T017
# T021 (refusal logic) - requires T015
```

---

## Parallel Example: Cross-Story (After Foundational)

If you have 4 developers, this is optimal parallelization:

```bash
# Developer 1: User Story 1 (MVP)
# Focus on T013-T021 (basic Q&A functionality)

# Developer 2: User Story 2 (Conversation Context)
# Focus on T022-T032 (session management, history)

# Developer 3: User Story 3 (Confidence Scoring)
# Start with T033-T034 (confidence models), stub search_textbook initially
# Complete after Developer 1 finishes T015

# Developer 4: User Story 4 (Error Handling)
# Focus on T042-T052 (retry logic, circuit breaker, error responses)
# Can integrate with US1 endpoint after Developer 1 finishes T017
```

---

## Implementation Strategy

### MVP-First Approach (Recommended)

**Sprint 1 (Week 1)**: Phase 1 (Setup) + Phase 2 (Foundational) + Phase 3 (User Story 1 - Basic Q&A)
- **Deliverable**: Working RAG agent that answers questions with citations
- **Validation**: Can ask "What is a ROS 2 node?" and get accurate, cited answer
- **Success Criteria**: SC-001 (90% factual correctness), SC-004 (100% citations), SC-009 (95% grounding)

**Sprint 2 (Week 2)**: Phase 4 (User Story 2 - Conversation Context) + Phase 5 (User Story 3 - Confidence Scoring)
- **Deliverable**: Multi-turn conversations + confidence-based refusal
- **Validation**: Can ask follow-ups, retrieve history, see confidence scores
- **Success Criteria**: SC-003 (95% refusal accuracy), SC-006 (90% context maintenance)

**Sprint 3 (Week 3)**: Phase 6 (User Story 4 - Error Handling) + Phase 7 (Streaming Support) + Phase 8 (Polish)
- **Deliverable**: Production-ready API with resilience + streaming
- **Validation**: Handles failures gracefully, streams responses in real-time
- **Success Criteria**: SC-002 (p95 < 3s), SC-005 (99.5% uptime), SC-007 (100 concurrent), SC-008 (actionable errors), SC-010 (15-min onboarding)

### Incremental Delivery Checkpoints

- **Checkpoint 1 (Phase 3 complete)**: MVP - Basic Q&A works, ready for user feedback
- **Checkpoint 2 (Phase 5 complete)**: Feature-complete agent - All core capabilities delivered
- **Checkpoint 3 (Phase 8 complete)**: Production-ready - Deployed and monitored

---

## Estimated Effort

**Total Implementation Time**: 10-14 hours (individual developer, sequential)

- Phase 1 (Setup): 0.5-1 hour
- Phase 2 (Foundational): 1-2 hours
- Phase 3 (User Story 1 - MVP): 2-3 hours ⭐ **CRITICAL PATH**
- Phase 4 (User Story 2): 1-2 hours
- Phase 5 (User Story 3): 1-1.5 hours
- Phase 6 (User Story 4): 1.5-2 hours
- Phase 7 (Streaming Support): 1-1.5 hours
- Phase 8 (Polish): 1-2 hours

**Parallel Team (4 developers)**: 4-6 hours (with proper coordination)

**MVP Delivery (Phase 1-3 only)**: 4-6 hours (sequential)

---

## Task Summary

**Total Tasks**: 67
- Phase 1 (Setup): 6 tasks
- Phase 2 (Foundational): 6 tasks (BLOCKING)
- Phase 3 (User Story 1 - MVP): 9 tasks
- Phase 4 (User Story 2): 11 tasks
- Phase 5 (User Story 3): 9 tasks
- Phase 6 (User Story 4): 11 tasks
- Phase 7 (Streaming Support): 6 tasks
- Phase 8 (Polish): 9 tasks

**Parallel Opportunities**: 18 tasks marked [P] (can run in parallel within their phase)

**User Story Breakdown**:
- US1 (Basic Q&A): 9 tasks → **MVP deliverable**
- US2 (Conversation Context): 11 tasks
- US3 (Confidence Scoring): 9 tasks
- US4 (Error Handling): 11 tasks

---

## Success Criteria Traceability

| Success Criteria | Validated By Tasks | Validation Method |
|------------------|-------------------|-------------------|
| **SC-001**: 90% factual correctness | T015 (search_textbook), T016 (agent), T017 (/chat/run), T021 (refusal) | Human evaluation (100 test questions) |
| **SC-002**: p95 latency < 3s | T017 (/chat/run async), T030 (agent caching), T044 (retry backoff) | pytest-benchmark + locust |
| **SC-003**: 95% refusal accuracy | T021 (refusal logic), T034 (confidence metrics), T035 (thresholds) | Human evaluation (out-of-domain test suite) |
| **SC-004**: 100% citation compliance | T018 (extract_sources), T020 (logging) | Automated test (/chat/run response.sources non-empty) |
| **SC-005**: 99.5% uptime | T042 (retry), T043 (circuit breaker), T052 (health check) | Uptime monitoring (production metrics) |
| **SC-006**: 90% context maintenance | T024-T026 (session persistence), T031 (summarization) | Multi-turn integration tests |
| **SC-007**: 100 concurrent requests | T009 (connection pooling), T012 (async FastAPI) | locust stress test |
| **SC-008**: 100% actionable errors | T046-T050 (error handling), T049 (ErrorResponse model) | Integration tests (400/429/503 scenarios) |
| **SC-009**: 95% grounding accuracy | T011 (strict RAG prompt), T015 (search_textbook), T034 (confidence) | Human evaluation (hallucination detection) |
| **SC-010**: 15-minute onboarding | T061 (validate quickstart.md) | User testing (new developer timer) |

---

## Notes

1. **No Tests**: The spec does not explicitly request tests, so test tasks are excluded. Focus is on implementation only. Tests can be added later if needed.

2. **MVP Scope**: User Story 1 (Phase 3) is the minimum viable product. Delivers core RAG functionality with citations and basic grounding. All other stories are enhancements.

3. **Parallel Execution**: If team capacity allows, User Stories 2, 3, 4 can start in parallel after Phase 2 completes. Coordinate on shared files (models.py, routers/chat.py).

4. **Streaming Support (Phase 7)**: Optional enhancement. Can be deferred if time-constrained. Requires US1 completion.

5. **Configuration First**: Phase 2 T007 (config.py) is critical - all subsequent tasks depend on environment variables being loaded correctly.

6. **Error Handling**: User Story 4 (Phase 6) tasks apply cross-cutting error handling to all endpoints. Should be completed before production deployment.

7. **Quickstart Validation**: Phase 8 T061 is critical - ensures quickstart.md instructions are accurate and achievable within 15 minutes (SC-010).

8. **Constitution Compliance**: Phase 8 T066 verifies all 7 constitution principles are met. This is a quality gate before marking feature complete.
