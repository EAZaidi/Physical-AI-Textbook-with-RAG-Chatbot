# Implementation Plan: RAG Agent API

**Branch**: `003-rag-agent-api` | **Date**: 2025-12-17 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/003-rag-agent-api/spec.md`

## Summary

Build a production-ready RAG agent that answers questions about the Physical AI & Humanoid Robotics textbook using OpenAI Agents SDK, Qdrant vector retrieval, and FastAPI. The agent will ensure zero hallucinations by grounding all responses strictly in retrieved book content, support multi-turn conversations, provide confidence scores for answer quality, and expose RESTful endpoints for integration. Key technical approach: use OpenAI Agents SDK's `@function_tool` decorator to wrap Qdrant semantic search, implement strict grounding through system instructions and guardrails, maintain conversation threads via SQLAlchemy sessions (PostgreSQL), and expose both synchronous (`/chat/run`) and streaming (`/chat/stream`) endpoints via FastAPI with async/await patterns.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**:
- `openai-agents-sdk` (Agent orchestration, tool integration, session management)
- `fastapi` (async REST API framework)
- `qdrant-client` (vector database client for semantic search)
- `openai` (embeddings API client - text-embedding-3-small model)
- `sqlalchemy` (session persistence via PostgreSQL)
- `pydantic` (request/response validation)
- `uvicorn` (ASGI server for FastAPI)
- `python-dotenv` (environment variable management)

**Storage**:
- **Vector Database**: Qdrant (cloud or self-hosted) - pre-populated with textbook embeddings from 001-rag-ingestion-pipeline
- **Session Storage**: PostgreSQL (via SQLAlchemySession) - conversation history, thread metadata
- **Environment Variables**: `.env` file for API keys (OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, DATABASE_URL)

**Testing**:
- `pytest` (unit tests for retrieval logic, confidence scoring)
- `pytest-asyncio` (async endpoint testing)
- `httpx` (async API client for integration tests)
- Human evaluation for hallucination detection (100 test questions per spec SC-001)

**Target Platform**: Linux server (Docker containerized for production, local development on macOS/Linux)

**Project Type**: Web application (backend API only - FastAPI service)

**Performance Goals**:
- p95 latency < 3000ms for single-turn questions (SC-002)
- 100 concurrent requests without >10% degradation (SC-007)
- 100 requests per minute per API key (rate limiting per FR-008)

**Constraints**:
- Zero hallucinations (95% grounding accuracy per SC-009)
- Mandatory source citations (100% compliance per SC-004)
- Retrieval threshold: similarity >= 0.4 (refuse to answer below threshold per FR-006)
- Conversation history: last 5 turns (10 messages total per assumption #9)
- Response length: max 1000 tokens (~750 words per assumption #7)

**Scale/Scope**:
- Initial load: <100 requests/hour, scaling to 1000 requests/hour within 6 months
- Qdrant collection: ~45-200 chunks from textbook (pre-existing from ingestion pipeline)
- Expected session count: 10-50 concurrent conversations
- API endpoints: 4 (chat/run, chat/stream, session history, session clear)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Source Accuracy & Verifiability (Principle I)
**Status**: PASS - All agent responses must be grounded in Qdrant-retrieved textbook chunks. System instructions enforce "ONLY use information from search_textbook tool". Guardrails validate responses against knowledge base. Source citations mandatory (FR-004, SC-004).

**Evidence**: Strict RAG instructions template (see research.md), confidence-based refusal logic when retrieval quality insufficient.

### ✅ Educational Clarity & Student Success (Principle II)
**Status**: PASS - Agent provides clear, pedagogical responses synthesized from textbook content. Confidence scores enable students to assess answer reliability. Citations guide students to specific textbook sections for deeper learning.

**Evidence**: Confidence scoring (FR-012, SC-003), disclaimer for low-confidence answers (US3 acceptance scenario 2).

### ✅ Reproducibility & Environment Consistency (Principle III)
**Status**: PASS - Dockerized FastAPI application with explicit dependencies (pyproject.toml/requirements.txt). Environment variables for all external services. Health checks validate Qdrant connectivity before startup.

**Evidence**: Docker Compose configuration (Phase 1), environment validation script, quickstart.md with setup instructions.

### ✅ Spec-Driven Content Development (Principle IV)
**Status**: PASS - Following Spec-Kit Plus workflow: spec.md → plan.md (this document) → tasks.md → implementation. Incremental validation via automated tests and human evaluation.

**Evidence**: This plan follows constitution-mandated workflow. Tasks.md will break down implementation into testable increments.

### ✅ RAG Chatbot Fidelity (Principle V)
**Status**: PASS - Core requirement of this feature. Agent explicitly refuses to answer when retrieval confidence < threshold. All responses cite specific chunks. "Not covered in this book" for out-of-domain queries.

**Evidence**: FR-003, FR-006, SC-003 (95% refusal accuracy), US1 acceptance scenario 3, multi-layer hallucination prevention (research.md section 4).

### ✅ Modular Architecture & Progressive Complexity (Principle VI)
**Status**: PASS - Agent leverages existing Module 1-7 content indexed in Qdrant. Retrieval spans all modules. Conversation context helps students explore topics progressively (e.g., "What is SLAM?" → "Can you explain the math?" → "Show me an example").

**Evidence**: Multi-turn conversation support (US2), context-aware retrieval, 5-turn history window.

### ✅ Production-Ready Deployment Standards (Principle VII)
**Status**: PASS - FastAPI application with Docker, health checks, structured logging, API key authentication, error handling, rate limiting. GitHub Actions CI/CD for deployment.

**Evidence**: FR-008 (rate limiting), FR-009 (logging), US4 (error handling), SC-005 (99.5% uptime target), Docker Compose configuration.

**Post-Design Re-Check**: ✅ PASS - All principles satisfied. No constitution violations detected.

## Project Structure

### Documentation (this feature)

```text
specs/003-rag-agent-api/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (OpenAI Agents SDK integration patterns)
├── data-model.md        # Phase 1 output (ChatRequest, ChatResponse, ConversationThread entities)
├── quickstart.md        # Phase 1 output (local setup, Docker deployment, API examples)
├── contracts/           # Phase 1 output (OpenAPI schema for /chat/* endpoints)
│   └── api-schema.yaml
├── checklists/          # Existing quality validation
│   └── requirements.md
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── agent_api/
│   ├── __init__.py
│   ├── main.py               # FastAPI application entry point
│   ├── agent.py              # Agent initialization, tool registration
│   ├── tools.py              # @function_tool definitions (search_textbook, etc.)
│   ├── models.py             # Pydantic models (ChatRequest, ChatResponse, etc.)
│   ├── sessions.py           # SQLAlchemySession setup, session management
│   ├── confidence.py         # ConfidenceMetrics calculation, scoring logic
│   ├── config.py             # Environment variables, settings (OPENAI_API_KEY, etc.)
│   └── routers/
│       ├── __init__.py
│       ├── chat.py           # /chat/run, /chat/stream endpoints
│       └── sessions.py       # /session/{id}/history, DELETE /session/{id}
│
├── tests/
│   ├── test_tools.py         # Unit tests for retrieval, confidence scoring
│   ├── test_agent.py         # Unit tests for agent grounding, refusal logic
│   ├── test_endpoints.py     # Integration tests for FastAPI endpoints
│   └── fixtures/
│       └── test_queries.json # 100 test questions for human evaluation (SC-001)
│
├── pyproject.toml            # uv project dependencies
├── Dockerfile                # Production container image
├── docker-compose.yml        # Local development stack (FastAPI + PostgreSQL)
├── .env.example              # Environment variable template
└── README.md                 # Updated with agent API usage instructions
```

**Structure Decision**: Web application structure selected because:
1. FastAPI backend is the sole component (no frontend in this feature)
2. Reuses existing `backend/` directory from 001-rag-ingestion-pipeline
3. Agent API lives in `backend/agent_api/` module for clear separation from ingestion code
4. Shared `backend/.env` and `backend/pyproject.toml` for unified Python environment

## Complexity Tracking

> **No constitution violations detected - this section is empty.**

## Architecture Decisions

### 1. OpenAI Agents SDK vs Custom Agent Framework

**Decision**: Use OpenAI Agents SDK

**Rationale**:
- Built-in conversation management (sessions) eliminates manual history handling
- `@function_tool` decorator simplifies Qdrant integration (auto-generates tool schemas)
- Streaming support via `run_stream_async()` enables real-time response delivery
- Guardrails integration available for hallucination detection
- Production-ready patterns documented (see research.md)

**Alternatives Considered**:
- **LangChain**: More complex API, requires custom RetrievalQA chains, less direct streaming support
- **Custom OpenAI API calls**: Requires manual function calling implementation, conversation history management, tool schema generation - significant engineering overhead
- **Haystack**: Strong for RAG but lacks native OpenAI Agents SDK features like sessions and streaming

**Trade-offs**: OpenAI Agents SDK is relatively new (2024 release), documentation still evolving, but benefits outweigh learning curve.

### 2. Qdrant Vector Database (Pre-Existing Choice)

**Decision**: Reuse Qdrant collection from 001-rag-ingestion-pipeline

**Rationale**:
- Collection already populated with textbook embeddings (1024-dim Cohere vectors)
- High-performance semantic search (cosine similarity, metadata filtering)
- Cloud-hosted or self-hosted options for production scaling
- Python client with async support for FastAPI integration

**Integration Pattern**: Wrap Qdrant search in `@function_tool` to expose as agent capability (see research.md section 1-2).

### 3. Session Management: SQLAlchemySession (PostgreSQL)

**Decision**: PostgreSQL via SQLAlchemySession for conversation persistence

**Rationale**:
- Multi-server deployment support (horizontal scaling)
- Persistent conversation history beyond server restarts
- Connection pooling for concurrent sessions
- ACID guarantees for thread safety

**Alternatives Considered**:
- **SQLite (in-memory)**: Fast for dev/test but no persistence
- **SQLite (file-based)**: Simple but poor concurrent access (file locking)
- **OpenAIConversationsSession**: Cloud-hosted by OpenAI but less control, potential vendor lock-in

**Configuration** (from research.md section 3):
```python
engine = create_engine(
    "postgresql://user:password@localhost:5432/chatbot_db",
    pool_size=20,       # Handle 20 concurrent sessions
    max_overflow=40,    # Burst to 60 total connections
    pool_pre_ping=True  # Verify connections before use
)
```

### 4. Hallucination Prevention: Multi-Layer Strategy

**Decision**: Implement 3-layer defense-in-depth

**Layer 1: System Instructions** - Strict RAG prompt enforcing tool-only responses
**Layer 2: Guardrails** - OpenAI hallucination detection API validating responses
**Layer 3: Confidence Filtering** - Refuse answers when retrieval quality < threshold

**Rationale**: Single-layer approaches fail in edge cases. Multi-layer ensures:
- Agent won't attempt to answer without retrieval (Layer 1)
- Even if agent tries to use prior knowledge, guardrails block it (Layer 2)
- Low-quality retrievals are rejected before generation (Layer 3)

**Details**: See research.md section 4 for implementation patterns.

### 5. FastAPI Endpoint Design: Dual Mode (Sync + Stream)

**Decision**: Expose both `/chat/run` (synchronous) and `/chat/stream` (SSE) endpoints

**Rationale**:
- `/chat/run`: Simpler integration for clients needing complete responses (mobile apps, batch processing)
- `/chat/stream`: Better UX for real-time applications (web UI, chatbot widgets)
- Minimal code duplication (both use same agent instance, different response handling)

**Streaming Implementation** (from research.md section 5):
- Server-Sent Events (SSE) with `text/event-stream` content type
- Event types: `raw_response` (text deltas), `tool_call` (retrieval events), `done` (completion with usage stats), `error` (failures)
- Nginx configuration: `X-Accel-Buffering: no` header to prevent proxy buffering

### 6. Confidence Scoring: Multi-Metric Approach

**Decision**: Calculate confidence using 5 metrics (not just average similarity)

**Metrics** (from research.md section 6):
1. **Average Similarity**: Mean cosine score across top-K chunks
2. **Min Similarity**: Weakest match (detects poor retrievals)
3. **Max Similarity**: Best match (detects single good result among noise)
4. **Num Chunks**: How many results returned (low count = sparse coverage)
5. **Chunk Diversity**: Semantic variance (high diversity = comprehensive topic coverage)

**Confidence Levels**:
- **High** (>0.8): avg >= 0.85, min >= 0.75, count >= 3 → Answer confidently
- **Medium** (0.5-0.8): avg >= 0.75, min >= 0.65, count >= 2 → Answer with disclaimer
- **Low** (<0.5): avg >= 0.65, count >= 1 → Refuse to answer (conservative)
- **Insufficient** (0): No results or avg < 0.65 → "No information found"

**Rationale**: Single-metric scoring fails to detect edge cases (e.g., one great match + four terrible matches yields high average but poor answer quality).

### 7. Rate Limiting Strategy

**Decision**: Per-API-key rate limiting (100 requests/minute default)

**Implementation**: Use FastAPI middleware with in-memory token bucket (dev) or Redis (production) for distributed rate limiting.

**Rationale**: Prevents abuse, ensures fair resource allocation, aligns with FR-008 requirement.

### 8. Error Handling & Resilience

**Decision**: Comprehensive error classes with actionable messages (per US4)

**Error Categories**:
- **400 Bad Request**: Invalid input (malformed JSON, query too long)
- **401 Unauthorized**: Invalid/missing API key
- **413 Payload Too Large**: Query exceeds 1000 characters
- **429 Too Many Requests**: Rate limit exceeded (with Retry-After header)
- **503 Service Unavailable**: Qdrant or OpenAI API unreachable (retry with exponential backoff)
- **500 Internal Server Error**: Unexpected failures (log full traceback, return generic message)

**Exponential Backoff** (for OpenAI rate limits): 1s, 2s, 4s, 8s, 16s (max 5 retries)

## File/Module Responsibilities

### backend/agent_api/main.py
- FastAPI application initialization
- Health check endpoint (`GET /health`)
- API key authentication middleware
- Rate limiting middleware
- CORS configuration
- Lifespan context (database connection setup/teardown)

### backend/agent_api/agent.py
- Agent instance creation with strict RAG instructions
- Tool registration (`search_textbook`, future tools)
- Session initialization (SQLAlchemySession per user)
- Agent caching (per session_id to avoid recreation overhead)

### backend/agent_api/tools.py
- `@function_tool` decorated functions
- `search_textbook(query, top_k)` - core retrieval from Qdrant
- Embedding generation (OpenAI text-embedding-3-small)
- Qdrant query execution (cosine similarity, score threshold filtering)
- Result parsing to `RetrievalResult` dataclass

### backend/agent_api/models.py
- Pydantic request/response models:
  - `ChatRequest` (message, session_id, stream flag)
  - `ChatResponse` (response, confidence, sources, session_id)
  - `RetrievalResult` (chunks, scores, average_score, has_sufficient_context)
  - `ConfidenceMetrics` (5-metric confidence assessment)
  - `SessionHistoryResponse` (conversation turns with timestamps)

### backend/agent_api/sessions.py
- SQLAlchemySession factory (`get_session(session_id)`)
- Database engine configuration (connection pooling)
- Session cleanup utilities (clear history, delete expired threads)
- Thread metadata management (creation time, last updated, topic summary)

### backend/agent_api/confidence.py
- `ConfidenceMetrics.calculate()` - multi-metric confidence scoring
- Adaptive threshold logic (query type-aware adjustment)
- Reranking support (optional CrossEncoder for precision)
- Explanation generation (human-readable confidence descriptions)

### backend/agent_api/config.py
- Environment variable loading (dotenv)
- Validated settings (API keys, URLs, thresholds)
- Constants (MAX_QUERY_LENGTH=1000, MAX_RESPONSE_TOKENS=1000, RATE_LIMIT=100)
- OpenAI/Qdrant client initialization

### backend/agent_api/routers/chat.py
- `POST /chat/run` - synchronous chat endpoint
- `POST /chat/stream` - SSE streaming endpoint
- Error handling (try-catch with specific HTTPException codes)
- Source extraction from agent response events

### backend/agent_api/routers/sessions.py
- `GET /session/{id}/history` - retrieve conversation turns
- `DELETE /session/{id}` - clear session and cache
- `GET /session/{id}/summary` - conversation topic summary (optional)

### backend/tests/test_tools.py
- Unit tests for `search_textbook` function
- Mock Qdrant client responses
- Confidence score calculation validation
- Edge cases (empty results, low scores, high diversity)

### backend/tests/test_agent.py
- Agent grounding verification (refuses to answer without retrieval)
- Citation compliance (responses include source references)
- Refusal logic (low confidence triggers "no information" message)
- Multi-turn context preservation (follow-up questions work correctly)

### backend/tests/test_endpoints.py
- Integration tests for `/chat/run` and `/chat/stream`
- Session isolation (concurrent conversations don't leak)
- Rate limiting enforcement (429 after 100 requests)
- Error handling (400/503/500 scenarios)

## Dependencies

### Python Packages (pyproject.toml)

```toml
[project]
dependencies = [
    "fastapi>=0.110.0",
    "uvicorn[standard]>=0.30.0",
    "openai-agents-sdk>=0.1.0",  # Agent orchestration
    "openai>=1.55.0",            # Embeddings API
    "qdrant-client>=1.12.0",     # Vector database
    "sqlalchemy>=2.0.0",         # Session persistence
    "pydantic>=2.0.0",           # Data validation
    "python-dotenv>=1.0.0",      # Environment variables
    "psycopg2-binary>=2.9.0",    # PostgreSQL driver
]

[project.optional-dependencies]
dev = [
    "pytest>=8.0.0",
    "pytest-asyncio>=0.24.0",
    "httpx>=0.28.0",             # Async API testing
    "pytest-cov>=6.0.0",         # Coverage reporting
]
```

### External Services

| Service | Purpose | Configuration |
|---------|---------|---------------|
| **Qdrant** | Vector database (semantic search) | URL, API key in `.env` (QDRANT_URL, QDRANT_API_KEY) |
| **OpenAI API** | Embeddings (text-embedding-3-small), Agent SDK | API key in `.env` (OPENAI_API_KEY) |
| **PostgreSQL** | Session persistence (conversation history) | Connection string in `.env` (DATABASE_URL) |

### Pre-Existing Dependencies

- **001-rag-ingestion-pipeline**: Qdrant collection `rag_embedding` must be populated with textbook chunks
- **Textbook Content**: Assumes Docusaurus site is built and indexed

## Implementation Phases

### Phase 0: Research & Technical Decisions ✅

**Status**: Complete (see research.md)

**Outputs**:
- `research.md` documenting OpenAI Agents SDK integration patterns
- Architecture decisions for session management, hallucination prevention, confidence scoring
- Code patterns for `@function_tool`, FastAPI streaming, multi-metric confidence

**Key Findings** (summarized above in Architecture Decisions section):
- Use SQLAlchemySession (PostgreSQL) for multi-server deployments
- Implement 3-layer hallucination prevention (instructions + guardrails + confidence filtering)
- Expose dual endpoints (sync `/chat/run` + streaming `/chat/stream`)
- Multi-metric confidence scoring (5 metrics instead of single average)

### Phase 1: Design & Contracts 🔄

**Objective**: Define data models, API contracts, and deployment configuration

**Tasks**:

1. **Generate data-model.md** (entity definitions from spec.md Key Entities section):
   - ChatRequest: `message: str, session_id: str, stream: bool, top_k: int`
   - ChatResponse: `response: str, confidence: float, sources: list[str], session_id: str, timestamp: str`
   - ConversationThread: `thread_id: str, messages: list, created_at: datetime, updated_at: datetime, topic_summary: str`
   - RetrievalResult: `chunks: list[str], scores: list[float], average_score: float, has_sufficient_context: bool`
   - ConfidenceMetrics: `average_similarity: float, min_similarity: float, max_similarity: float, num_chunks: int, chunk_diversity: float, confidence_level: str, should_answer: bool`

2. **Generate API contracts** (`contracts/api-schema.yaml` - OpenAPI 3.1 spec):
   - `POST /chat/run` → `ChatRequest` body → `ChatResponse` JSON
   - `POST /chat/stream` → `ChatRequest` body → SSE stream (events: raw_response, tool_call, done, error)
   - `GET /session/{session_id}/history` → `SessionHistoryResponse` JSON
   - `DELETE /session/{session_id}` → 204 No Content
   - Error schemas (400, 401, 413, 429, 503, 500)

3. **Generate quickstart.md** (setup and usage guide):
   - Prerequisites (Python 3.11+, uv, PostgreSQL, Qdrant access)
   - Local development setup (`uv sync`, `docker-compose up`)
   - Environment variables (`.env` template)
   - API examples (curl commands for /chat/run, /chat/stream)
   - Testing instructions (`pytest`, human evaluation for hallucination)

4. **Update agent context** (run `.specify/scripts/powershell/update-agent-context.ps1`):
   - Add OpenAI Agents SDK, FastAPI, SQLAlchemy to technology context
   - Preserve manual additions (existing ROS 2, Gazebo, etc.)

**Outputs**:
- `data-model.md` (entity schemas with field descriptions)
- `contracts/api-schema.yaml` (OpenAPI specification)
- `quickstart.md` (developer onboarding guide)
- Updated `.claude/` or agent-specific context file

**Acceptance Criteria**:
- All 5 entities from spec.md documented with field types and relationships
- OpenAPI schema validates with Swagger Editor (no errors)
- Quickstart enables new developer to run `/chat/run` endpoint in < 15 minutes (SC-010)
- Agent context includes new technologies without overwriting manual additions

### Phase 2: Task Breakdown (Not Part of /sp.plan)

**Note**: This phase is handled by the separate `/sp.tasks` command, which will:
- Generate `tasks.md` with testable implementation tasks
- Organize tasks by user story (P1-P4) for incremental delivery
- Define acceptance criteria for each task

**Expected Task Categories** (preview for context):
- **Setup**: Project structure, dependencies, environment configuration
- **User Story 1 (P1 - MVP)**: Basic retrieval tool, agent initialization, /chat/run endpoint, grounding validation
- **User Story 2 (P2)**: SQLAlchemySession setup, conversation history persistence, multi-turn context handling
- **User Story 3 (P3)**: Confidence scoring implementation, low-confidence refusal logic
- **User Story 4 (P4)**: Error handling, rate limiting, health checks, exponential backoff retry
- **Polish**: Documentation, Docker configuration, deployment guide, human evaluation test suite

## Validation Strategy

### Unit Testing (pytest)

**Coverage Targets**:
- `tools.py`: 90% coverage (retrieval logic, confidence calculation)
- `agent.py`: 80% coverage (grounding validation, session management)
- `confidence.py`: 95% coverage (multi-metric scoring, edge cases)

**Key Test Cases**:
- Retrieval returns empty results → confidence=0, should_answer=False
- Retrieval returns low scores (<0.4) → refuses to answer
- Retrieval returns high scores (>0.85) → confident answer
- Multi-turn conversation preserves context (follow-up questions work)
- Agent refuses to answer without calling search_textbook first

### Integration Testing (pytest + httpx)

**Endpoint Tests**:
- `POST /chat/run` with valid question → 200 OK with citations
- `POST /chat/run` with out-of-domain question → 200 OK with "no information found"
- `POST /chat/stream` returns SSE events (raw_response, tool_call, done)
- `GET /session/{id}/history` returns conversation turns
- `DELETE /session/{id}` clears session (subsequent GET returns empty)
- Rate limit enforcement (101st request in 1 minute → 429 Too Many Requests)

### Human Evaluation (SC-001, SC-009)

**Test Suite**: 100 questions covering:
- **In-domain questions** (60): Topics covered in textbook (ROS 2, URDF, Gazebo, VLA)
- **Out-of-domain questions** (20): Unrelated topics (quantum computing, cooking)
- **Edge cases** (20): Ambiguous, contradictory context, very long questions

**Metrics**:
- **Factual Correctness**: Manual review of answers against textbook (target: 90% per SC-001)
- **Hallucination Rate**: Identify responses containing information NOT in retrieved chunks (target: <5% per SC-009)
- **Refusal Accuracy**: Out-of-domain questions correctly refused (target: 95% per SC-003)
- **Citation Quality**: All claims have source references (target: 100% per SC-004)

**Evaluation Process**:
1. Run all 100 test questions through `/chat/run` endpoint
2. For each response, human evaluator checks:
   - Are facts correct per textbook?
   - Are all claims cited?
   - Does response contain hallucinated info?
   - Was refusal appropriate (for out-of-domain)?
3. Calculate metrics and compare against success criteria

### Performance Testing (SC-002, SC-007)

**Latency Measurement**:
- Use `pytest-benchmark` to measure p50, p95, p99 latency for `/chat/run`
- Target: p95 < 3000ms (SC-002)
- Measure cold start vs warm start (agent caching impact)

**Load Testing**:
- Use `locust` to simulate 100 concurrent users
- Verify response time degradation < 10% at 100 concurrent requests (SC-007)
- Identify bottlenecks (Qdrant queries, OpenAI API calls, database sessions)

## Deployment Configuration

### Docker Setup

**Dockerfile** (multi-stage build):
```dockerfile
FROM python:3.11-slim AS base
WORKDIR /app
COPY pyproject.toml uv.lock ./
RUN pip install uv && uv sync --no-dev

FROM base AS runtime
COPY backend/agent_api ./agent_api
EXPOSE 8000
CMD ["uvicorn", "agent_api.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

**docker-compose.yml** (local development):
```yaml
version: '3.8'
services:
  agent-api:
    build: .
    ports:
      - "8000:8000"
    environment:
      - DATABASE_URL=postgresql://user:password@postgres:5432/chatbot_db
      - QDRANT_URL=${QDRANT_URL}
      - QDRANT_API_KEY=${QDRANT_API_KEY}
      - OPENAI_API_KEY=${OPENAI_API_KEY}
    depends_on:
      - postgres

  postgres:
    image: postgres:16
    environment:
      POSTGRES_USER: user
      POSTGRES_PASSWORD: password
      POSTGRES_DB: chatbot_db
    volumes:
      - postgres_data:/var/lib/postgresql/data

volumes:
  postgres_data:
```

### Environment Variables (.env.example)

```env
# OpenAI API
OPENAI_API_KEY=sk-proj-...

# Qdrant Vector Database
QDRANT_URL=https://your-qdrant-instance.cloud.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# PostgreSQL Database
DATABASE_URL=postgresql://user:password@localhost:5432/chatbot_db

# Agent Configuration
MAX_QUERY_LENGTH=1000
MAX_RESPONSE_TOKENS=1000
RETRIEVAL_TOP_K=5
SIMILARITY_THRESHOLD=0.4
CONFIDENCE_THRESHOLD=0.75

# API Configuration
RATE_LIMIT_PER_MINUTE=100
API_KEY=your-api-key-for-auth
```

## Risk Mitigation

### Risk 1: OpenAI API Rate Limits

**Likelihood**: Medium (100+ concurrent users could exceed quota)

**Impact**: High (service unavailable, poor user experience)

**Mitigation**:
- Implement exponential backoff retry (1s, 2s, 4s, 8s, 16s)
- Monitor OpenAI usage dashboard for quota warnings
- Cache agent instances per session_id (reduce overhead)
- Consider upgrading to higher OpenAI tier for production

**Fallback**: Display "Service temporarily unavailable, please retry" with Retry-After header

### Risk 2: Qdrant Service Downtime

**Likelihood**: Low (cloud-hosted Qdrant has high uptime SLA)

**Impact**: Critical (retrieval fails, agent cannot answer any questions)

**Mitigation**:
- Health check endpoint (`GET /health`) validates Qdrant connectivity on startup
- Circuit breaker pattern: after 5 consecutive Qdrant failures, return 503 for 60 seconds
- Monitor Qdrant status page for scheduled maintenance

**Fallback**: Return HTTP 503 with message "Vector search service unavailable, please retry"

### Risk 3: Hallucination Despite Guardrails

**Likelihood**: Low (multi-layer prevention, but edge cases possible)

**Impact**: Critical (undermines trust, violates constitution)

**Mitigation**:
- Human evaluation on 100-question test suite before production (SC-001)
- Continuous monitoring: log all queries + responses for spot-checking
- User feedback mechanism ("Was this answer helpful?") to flag bad responses
- Quarterly re-evaluation with new test questions

**Fallback**: If hallucination rate > 5%, revert to more conservative confidence thresholds (raise from 0.75 to 0.85)

### Risk 4: PostgreSQL Connection Pool Exhaustion

**Likelihood**: Medium (100 concurrent requests with pool_size=20)

**Impact**: Medium (new sessions fail to connect, HTTP 503)

**Mitigation**:
- Configure pool_size=20, max_overflow=40 (60 total connections)
- Enable `pool_pre_ping` to detect stale connections
- Monitor connection pool metrics (active, idle, overflow)
- Set connection timeout (10 seconds) to fail fast

**Fallback**: Return HTTP 503 with Retry-After: 5 header

## Success Criteria Traceability

| Success Criteria | Implementation Evidence | Validation Method |
|------------------|------------------------|-------------------|
| **SC-001**: 90% factual correctness | Multi-layer hallucination prevention (research.md §4), strict RAG instructions | Human evaluation (100 test questions) |
| **SC-002**: p95 latency < 3000ms | Async FastAPI endpoints, agent caching, Qdrant optimized queries | pytest-benchmark + locust load tests |
| **SC-003**: 95% refusal accuracy | Confidence scoring (confidence.py), threshold filtering | Human evaluation (out-of-domain test suite) |
| **SC-004**: 100% citation compliance | Source extraction from agent events (routers/chat.py) | Automated test (check `sources` field non-empty) |
| **SC-005**: 99.5% uptime | Health checks, error handling, retry logic, Docker deployment | Uptime monitoring (production metrics) |
| **SC-006**: 90% context maintenance | SQLAlchemySession (5-turn history), conversation summarization | Multi-turn integration tests (follow-up questions) |
| **SC-007**: 100 concurrent requests | Connection pooling (pool_size=20), async/await patterns | locust stress test (100 concurrent users) |
| **SC-008**: 100% actionable errors | Comprehensive HTTPException handling (US4) | Integration tests (400/429/503 scenarios) |
| **SC-009**: 95% grounding accuracy | Guardrails validation, confidence filtering | Human evaluation (hallucination detection) |
| **SC-010**: 15-minute onboarding | quickstart.md with step-by-step setup, curl examples | User testing (new developer timer) |

## Post-Implementation Checklist

- [ ] All 10 success criteria validated (SC-001 through SC-010)
- [ ] 100-question human evaluation complete (>90% factual correctness, <5% hallucinations)
- [ ] Integration tests passing (endpoints, sessions, error handling)
- [ ] Performance tests passing (p95 < 3s, 100 concurrent requests)
- [ ] Docker image builds successfully (`docker build -t agent-api .`)
- [ ] quickstart.md tested (new developer onboarding < 15 minutes)
- [ ] API documentation complete (OpenAPI schema, curl examples)
- [ ] Environment variable validation (startup fails with clear error if missing keys)
- [ ] Health check endpoint functional (`GET /health` returns Qdrant status)
- [ ] Rate limiting enforced (101st request → HTTP 429)
- [ ] Constitution compliance verified (all 7 principles PASS)
