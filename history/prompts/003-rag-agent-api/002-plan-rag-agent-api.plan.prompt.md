---
id: 002
title: Plan RAG Agent API Implementation
stage: plan
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5
feature: 003-rag-agent-api
branch: 003-rag-agent-api
user: USER
command: /sp.plan
labels: ["architecture", "rag-agent", "openai-agents-sdk", "fastapi", "planning"]
links:
  spec: specs/003-rag-agent-api/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-rag-agent-api/research.md
  - specs/003-rag-agent-api/plan.md
  - specs/003-rag-agent-api/data-model.md
  - specs/003-rag-agent-api/contracts/api-schema.yaml
  - specs/003-rag-agent-api/quickstart.md
  - CLAUDE.md
tests:
  - Constitution compliance check (7/7 PASS)
  - Spec requirements coverage validation
  - Architecture decision completeness review
---

## Prompt

User executed: `/sp.plan`

Then continued with: `/sp.plan resume to the point where the sp.plan was left off`

Context: Planning implementation for feature 003-rag-agent-api (RAG Agent Development), which builds a production-ready RAG agent that answers questions about the Physical AI & Humanoid Robotics textbook using OpenAI Agents SDK, Qdrant vector retrieval, and FastAPI.

## Response snapshot

**Phase 0 - Research**: Created comprehensive research.md with 7 key architectural decisions:
1. OpenAI Agents SDK `@function_tool` pattern for search_textbook retrieval
2. SQLAlchemySession (PostgreSQL) for conversation persistence
3. Multi-layer hallucination prevention (system instructions + Guardrails API + confidence filtering)
4. FastAPI dual endpoint design (sync `/chat/run` + streaming `/chat/stream`)
5. Multi-metric confidence scoring (5 metrics vs single average)
6. Qdrant collection configuration reuse from 001-rag-ingestion-pipeline
7. Comprehensive error handling with exponential backoff and circuit breakers

**Phase 1 - Design Artifacts Generated**:
1. ✅ data-model.md: Entity definitions for ChatRequest, ChatResponse, ConversationThread, RetrievalResult, ConfidenceMetrics
2. ✅ contracts/api-schema.yaml: OpenAPI 3.1 spec with REST endpoints and SSE streaming
3. ✅ quickstart.md: Developer onboarding guide with environment setup and API examples
4. ✅ Agent context updated: Added Python 3.11+ to CLAUDE.md technology context

**Architecture Highlights**:
- Backend structure: backend/agent_api/ with main.py, agent.py, tools.py, models.py, sessions.py, confidence.py, config.py
- Dual endpoint strategy: Synchronous for simple requests, streaming for real-time token delivery
- Zero-hallucination focus: 95% grounding accuracy target (SC-009) via multi-layer prevention
- Multi-metric confidence: average_similarity, min_similarity, max_similarity, num_chunks, chunk_diversity
- Session management: SQLAlchemy with PostgreSQL, connection pooling (pool_size=20, max_overflow=40)
- Error resilience: OpenAI rate limit retry (exponential backoff), Qdrant circuit breaker, comprehensive HTTP error taxonomy

**Constitution Check**: All 7 principles PASS
- ✅ Source Accuracy & Verifiability: All retrieval grounded in indexed textbook chunks
- ✅ Educational Clarity & Student Success: Clear API documentation and error messages
- ✅ Reproducibility & Environment Consistency: Dockerized backend with explicit dependency versions
- ✅ Spec-Driven Content Development: Following full Spec-Kit Plus workflow
- ✅ RAG Chatbot Fidelity: Zero-hallucination architecture with citation enforcement
- ✅ Modular Architecture & Progressive Complexity: Clean separation (tools, models, sessions, confidence)
- ✅ Production-Ready Deployment Standards: Health checks, logging, secure env var management

## Outcome

- ✅ Impact: Complete implementation plan (plan.md) with 692 lines covering architecture, file responsibilities, dependencies, validation strategy, deployment config, risk mitigation, and success criteria traceability. All Phase 1 design artifacts generated and validated.
- 🧪 Tests: Constitution compliance (7/7 PASS), spec coverage validation, architecture decision completeness review
- 📁 Files: Created research.md, plan.md, data-model.md, contracts/api-schema.yaml, quickstart.md; updated CLAUDE.md
- 🔁 Next prompts: Run `/sp.tasks` to generate task breakdown (tasks.md) from plan.md for Phase 2 implementation
- 🧠 Reflection: Research agent successfully consolidated OpenAI Agents SDK patterns and Qdrant integration strategies. Multi-layer hallucination prevention architecture aligns with constitution principle V (RAG Chatbot Fidelity). Dual endpoint design (sync/stream) provides flexibility for different frontend integration patterns.

## Evaluation notes (flywheel)

- Failure modes observed: None. PowerShell encoding warning during context update (non-blocking).
- Graders run and results (PASS/FAIL): Constitution check PASS (7/7), Spec coverage PASS (all 12 FR requirements addressed in plan), Architecture completeness PASS (file structure, dependencies, validation strategy all defined)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A - planning phase complete, ready for task generation
