---
id: 002
title: Create RAG Chatbot Frontend Integration Plan
stage: plan
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: 004-rag-chatbot-frontend
branch: 004-rag-chatbot-frontend
user: USER
command: /sp.plan
labels: ["planning", "architecture", "frontend", "chatbot", "integration", "rag"]
links:
  spec: ../specs/004-rag-chatbot-frontend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-rag-chatbot-frontend/plan.md
  - specs/004-rag-chatbot-frontend/research.md
  - specs/004-rag-chatbot-frontend/data-model.md
  - specs/004-rag-chatbot-frontend/quickstart.md
  - specs/004-rag-chatbot-frontend/contracts/backend-api.md
tests: []
---

## Prompt

```text
- Add frontend client to call FastAPI RAG endpoints
- Pass user query and optional selected text
- Handle CORS and local configuration
- Render agent responses in the book UI
```

## Response snapshot

Created comprehensive implementation plan for RAG Chatbot Frontend Integration (feature 004-rag-chatbot-frontend) following the `/sp.plan` workflow.

**Planning Workflow Executed**:

**Phase 0: Research** ✅
- Created `research.md` with 10 technical decisions documented
- Decision 1: Docusaurus React Components (vs iframe/extension)
- Decision 2: React Context + sessionStorage for state management
- Decision 3: Native fetch API with custom hooks
- Decision 4: react-markdown for rendering
- Decision 5: window.getSelection() + Cmd/Ctrl+K for text selection
- Decision 6: EventSource API with @microsoft/fetch-event-source for streaming
- Decision 7: Environment variables via Docusaurus customFields
- Decision 8: CORS already handled by backend (003-rag-agent-api)
- Decision 9: Accessibility with ARIA labels, keyboard navigation (WCAG 2.1 AA)
- Decision 10: Responsive design (full screen <768px, sidebar >768px)

**Phase 1: Design & Contracts** ✅
- Created `data-model.md` with TypeScript interfaces:
  - ChatMessage (role, content, timestamp, confidence, sources, isStreaming)
  - ChatSession (sessionId, messages, createdAt, lastUpdatedAt, selectedContext, isLoading, error)
  - Source (chunkText, similarityScore, chapter, section, url, chunkIndex)
  - BackendConfig (baseUrl, timeoutMs, retryAttempts, apiKey, streamingEnabled)
  - ChatRequest, ChatResponse, StreamEvent types
  - Error types and validation rules
- Created `contracts/backend-api.md` with complete API documentation:
  - POST /chat/run (synchronous endpoint)
  - POST /chat/stream (SSE streaming endpoint)
  - GET /health (health check)
  - Request/response schemas, error handling, rate limiting
  - SSE protocol details with event types (tool_call, retrieval, content, done, error)
  - Configuration examples, security considerations
- Created `quickstart.md` with step-by-step developer guide:
  - 11 steps from environment setup to working chatbot (< 30 minutes target)
  - Includes dependency installation, configuration, component implementation
  - 5 manual test cases covering all 4 user stories
  - Troubleshooting guide with 5 common issues and solutions

**Phase 2: Complete Plan** ✅
- Created comprehensive `plan.md` (1258 lines) with all sections:
  - Summary with technical approach and MVP scope
  - Technical context (TypeScript 5.x, React 18.x, Docusaurus 3.x, dependencies)
  - Constitution check: All 7 principles passed ✅
  - Project structure (12-15 new TypeScript files in docs/src/components/RAGChatbot/)
  - Component architecture (hierarchy, responsibilities, 10 components documented)
  - State management strategy (React Context API with sessionStorage)
  - API client design (synchronous and streaming with fallback)
  - Text selection implementation (Cmd/Ctrl+K keyboard shortcut)
  - Markdown rendering (react-markdown + remark-gfm)
  - Mobile responsiveness (breakpoints, touch interactions, layout adjustments)
  - Accessibility implementation (keyboard navigation, ARIA labels, screen reader support)
  - Error handling strategy (5 error types with user-friendly messages)
  - Performance optimization (lazy loading, memoization, bundle size, caching)
  - Testing strategy (unit, integration, E2E tests)
  - Security considerations (input sanitization, API key management, CORS, session security)
  - Deployment and configuration (environment variables, build config, deployment steps)
  - Success criteria mapping (all 10 SC mapped to implementation strategies)
  - Risk analysis (5 risks identified with mitigation and fallback)
  - Implementation phases (7 phases from setup to documentation)
  - Technical decision log (references all 10 decisions from research.md)

**Key Architectural Decisions**:
1. **Framework**: Docusaurus React components (not iframe/extension) for tight integration
2. **State**: React Context API (not Redux/Zustand) - simpler, no external dependencies
3. **API Client**: Native fetch (not axios) with custom retry logic and timeout handling
4. **Streaming**: EventSource API with @microsoft/fetch-event-source for POST SSE support
5. **Persistence**: sessionStorage (not localStorage) for browser-session-scoped history
6. **Accessibility**: WCAG 2.1 AA compliance with full keyboard navigation and ARIA labels
7. **Mobile**: Responsive design (full-screen <768px, sidebar ≥768px)

**Project Scope**:
- 4 user stories (P1-P4): Basic Q&A, Text selection, Conversation history, Streaming
- 15 functional requirements, 10 success criteria
- ~12-15 TypeScript files, ~1500-2000 lines of code
- MVP (P1): Basic chatbot Q&A with citations
- Estimated: 40-60 hours (1-2 weeks for 1 developer)

**Constitution Compliance**: All 7 principles passed (Feature Quality, Code Quality, Testing, Performance, Security, Architecture, Documentation)

## Outcome

- ✅ Impact: Created complete implementation plan for RAG chatbot frontend integration. Documented 10 technical decisions, 5 design artifacts (plan.md, research.md, data-model.md, quickstart.md, contracts/backend-api.md), component architecture with 10 components, state management strategy, API client design, accessibility implementation, mobile responsiveness, error handling, performance optimization, testing strategy, security considerations, and deployment configuration. Plan maps all 4 user stories and 15 functional requirements to concrete implementation phases. All 7 constitution principles verified.
- 🧪 Tests: Planning phase includes comprehensive testing strategy. Unit tests: Components (ChatMessage, ChatInput, SourceCitation, ConfidenceBadge, ErrorMessage), hooks (useChatContext, useTextSelection), API client (sendChatMessage, streamChatMessage, error handling), utilities (session ID generation, input validation). Integration tests: 6 user flows (open/ask/receive, text selection, follow-up, clear conversation, navigate, error handling), accessibility (keyboard navigation, screen reader, focus management), responsive (desktop/mobile/touch). E2E tests: Complete Q&A cycle with real backend, streaming test, error handling test. All tests mapped to success criteria.
- 📁 Files: 5 files created: plan.md (1258 lines, complete implementation plan), research.md (10 technical decisions), data-model.md (TypeScript interfaces with validation rules), quickstart.md (11-step developer guide), contracts/backend-api.md (API contract documentation). All files in specs/004-rag-chatbot-frontend/.
- 🔁 Next prompts: (1) `/sp.tasks` to generate granular task breakdown from plan.md, breaking down into Phase 1 (Setup), Phase 2 (Core Components - MVP), Phase 3 (Text Selection), Phase 4 (Conversation History), Phase 5 (Streaming), Phase 6 (Polish), Phase 7 (Testing). (2) `/sp.implement` to execute implementation starting with MVP (P1 - Basic Q&A). (3) Optional: `/sp.adr` to document architectural decisions if needed (no significant decisions requiring ADR at this stage - all decisions documented in research.md).
- 🧠 Reflection: Planning phase followed systematic workflow: Research (10 technical unknowns resolved) → Data Model (TypeScript interfaces with validation) → API Contracts (backend integration fully specified) → Quickstart (developer onboarding guide) → Complete Plan (comprehensive architecture and implementation strategy). All decisions justified with alternatives considered. Component architecture is clear with 10 distinct responsibilities. State management uses React Context (simpler than Redux for this scope). API client handles both sync and streaming with graceful fallback. Accessibility and mobile responsiveness are first-class concerns (not afterthoughts). Error handling covers 5 error types with user-friendly messages. Testing strategy is comprehensive (unit/integration/E2E). Security considerations documented (no sensitive data in client, API key in env vars, CORS enforced by backend). Risk analysis identifies 5 risks with mitigation/fallback. Estimated 40-60 hours matches scope (12-15 files, 4 user stories, 15 requirements). Plan is complete and ready for task breakdown.

## Evaluation notes (flywheel)

- Failure modes observed: None. Planning workflow executed smoothly. All sections filled systematically. No ambiguities or missing decisions. Constitution check passed all 7 principles on first attempt. No clarifications needed - spec was complete and unambiguous.
- Graders run and results (PASS/FAIL): ✅ PASS - All 7 constitution principles passed. No violations. Principle 1 (Feature Quality): All user stories testable, success criteria measurable. Principle 2 (Code Quality): TypeScript interfaces, clear boundaries, single-responsibility components. Principle 3 (Testing): Unit/integration/E2E tests documented. Principle 4 (Performance): Streaming, lazy loading, memoization strategies. Principle 5 (Security): API key management, input validation, CORS. Principle 6 (Architecture): Clear separation (UI/state/API client), Context API, fallback strategies. Principle 7 (Documentation): 5 comprehensive artifacts created.
- Prompt variant (if applicable): N/A - Standard /sp.plan workflow
- Next experiment (smallest change to try): Monitor if React Context API scales as conversation history grows (currently limited to 100 messages). If performance degrades, consider migrating to Zustand (minimal API change) or implementing virtual scrolling for message list (react-window). Current approach should handle MVP scope (10-100 concurrent users). Test with 100+ messages in E2E testing phase to validate.

