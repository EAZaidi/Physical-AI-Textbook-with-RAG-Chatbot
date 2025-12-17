---
description: "Task breakdown for RAG Chatbot Frontend Integration"
---

# Tasks: RAG Chatbot Frontend Integration

**Input**: Design documents from `specs/004-rag-chatbot-frontend/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/backend-api.md ✅, quickstart.md ✅

**Tests**: Not explicitly requested in specification. Test tasks are excluded from this breakdown. Testing will be performed manually per quickstart.md and acceptance scenarios in spec.md.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

This is a **frontend integration feature** for an existing Docusaurus site:
- **Docusaurus root**: `docs/`
- **Components**: `docs/src/components/RAGChatbot/`
- **Theme**: `docs/src/theme/`
- **Static assets**: `docs/static/`
- **Config**: `docs/docusaurus.config.js`
- **Environment**: `docs/.env.local`, `.env.example`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization, dependencies, and configuration

- [x] T001 Navigate to docs/ directory and verify Docusaurus project exists
- [x] T002 Install chatbot dependencies: react-markdown@9.x, @microsoft/fetch-event-source@2.x, remark-gfm@4.x
- [x] T003 [P] Create environment file docs/.env.local with RAG_BACKEND_URL=http://localhost:8000
- [x] T004 [P] Create example environment file .env.example with RAG_BACKEND_URL and RAG_API_KEY placeholders
- [x] T005 [P] Update docs/docusaurus.config.js to add customFields: ragBackendUrl and ragApiKey from env vars
- [x] T006 Create directory docs/src/components/RAGChatbot/ for chatbot components
- [x] T007 [P] Create directory docs/static/img/ if not exists, add chat-icon.svg placeholder

**Checkpoint**: Dependencies installed, environment configured, directory structure ready

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core TypeScript interfaces and utilities that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T008 Create TypeScript interfaces in docs/src/components/RAGChatbot/types.ts (ChatMessage, ChatSession, Source, BackendConfig, ChatRequest, ChatResponse, StreamEvent types)
- [x] T009 [P] Create utility functions in docs/src/components/RAGChatbot/utils.ts (generateSessionId using crypto.randomUUID, validateMessage, validateSessionId)
- [x] T010 [P] Create API client synchronous function in docs/src/components/RAGChatbot/api.ts (sendChatMessage with fetch, timeout, error handling, retry logic)
- [x] T011 Create React Context structure in docs/src/components/RAGChatbot/ChatContext.tsx (ChatContextType interface, createContext call)
- [x] T012 [P] Create basic styles file in docs/src/components/RAGChatbot/styles.module.css with CSS variables and reset

**Checkpoint**: Foundation ready - all user story implementations can now begin in parallel

---

## Phase 3: User Story 1 - Basic Chatbot Interaction (Priority: P1) 🎯 MVP

**Goal**: Enable readers to ask questions and receive grounded answers with citations directly in the browser

**Independent Test**: Open any chapter page, type "What is a ROS 2 node?" in the chatbot input, submit query. Should receive a response with citations from the textbook within 5 seconds. Can be fully demonstrated without any other features.

### Implementation for User Story 1

- [x] T013 [P] [US1] Create ChatProvider component in docs/src/components/RAGChatbot/ChatContext.tsx (manages ChatSession state, loads from sessionStorage, provides sendMessage/clearHistory/setSelectedContext methods)
- [x] T014 [P] [US1] Create ChatToggle component in docs/src/components/RAGChatbot/ChatToggle.tsx (floating button bottom-right, onClick toggles panel, keyboard accessible)
- [x] T015 [P] [US1] Create ChatPanel component in docs/src/components/RAGChatbot/ChatPanel.tsx (container with header, message list, error display, input, conditional render when open)
- [x] T016 [P] [US1] Create ChatMessage component in docs/src/components/RAGChatbot/ChatMessage.tsx (renders user/assistant messages, role-specific styling, timestamp)
- [x] T017 [P] [US1] Create ChatInput component in docs/src/components/RAGChatbot/ChatInput.tsx (textarea with auto-resize, character counter, submit on Enter, disabled when loading)
- [x] T018 [P] [US1] Create LoadingIndicator component in docs/src/components/RAGChatbot/LoadingIndicator.tsx (animated spinner, "Searching textbook..." text, aria-live)
- [x] T019 [P] [US1] Create ErrorMessage component in docs/src/components/RAGChatbot/ErrorMessage.tsx (displays user-friendly error, retry button if retryable, dismiss button, role="alert")
- [x] T020 [P] [US1] Create SourceCitation component in docs/src/components/RAGChatbot/SourceCitation.tsx (displays chapter/section, similarity score, clickable link, expandable chunk excerpt)
- [x] T021 [US1] Integrate react-markdown in ChatMessage component for assistant messages (add ReactMarkdown with remarkGfm plugin, basic formatting only)
- [x] T022 [US1] Implement sendMessage method in ChatProvider (append user message, set loading, call sendChatMessage API, append assistant message, handle errors, save to sessionStorage)
- [x] T023 [US1] Implement sessionStorage persistence in ChatProvider (load session on mount, save on every message update, STORAGE_KEY='rag-chatbot-session')
- [x] T024 [US1] Create main RAGChatbot component in docs/src/components/RAGChatbot/index.tsx (wraps ChatProvider, renders ChatToggle and ChatPanel, exports as default)
- [x] T025 [US1] Create or update docs/src/theme/Root.tsx (wrap children with ChatProvider, pass backend config from Docusaurus customFields, render RAGChatbot globally)
- [x] T026 [US1] Add responsive CSS for mobile in docs/src/components/RAGChatbot/styles.module.css (full-screen <768px, sidebar ≥768px, touch targets 48x48px)
- [x] T027 [US1] Add error handling CSS in docs/src/components/RAGChatbot/styles.module.css (error message styling, retry/dismiss buttons)
- [x] T028 [US1] Implement clearHistory method in ChatProvider (generate new sessionId, reset messages array, clear error/selectedContext, save to sessionStorage)
- [x] T029 [US1] Add "New Conversation" button to ChatPanel header (calls clearHistory on click)
- [x] T030 [US1] Test basic Q&A flow per quickstart.md Step 10 (start dev server, open chatbot, ask question, verify response with citations)

**Checkpoint**: At this point, User Story 1 (Basic Q&A) should be fully functional and testable independently. This is the MVP!

---

## Phase 4: User Story 2 - Text Selection Context (Priority: P2)

**Goal**: Enable readers to select text on page and ask questions specifically about that selection

**Independent Test**: Select a paragraph about URDF, press Cmd+K (Mac) or Ctrl+K (Windows), chatbot opens with selected text pre-loaded as context. Ask "Can you explain this in simpler terms?" Should receive an answer specifically addressing the selected content.

### Implementation for User Story 2

- [x] T031 [US2] Create text selection event listener in docs/src/components/RAGChatbot/index.tsx (listen for Cmd/Ctrl+K keydown, capture window.getSelection(), validate max 5000 chars, set selectedContext, open panel, focus input)
- [x] T032 [US2] Add selectedContext display chip in ChatInput component (show above textarea when selectedContext present, truncate to 100 chars with tooltip, dismiss button with X, clear selectedContext on click)
- [x] T033 [US2] Update sendMessage in ChatProvider to include context field (if selectedContext present, add to ChatRequest as context field, send to backend)
- [x] T034 [US2] Add selectedContext clearing logic (clear when user closes chatbot, clear when user dismisses chip)
- [x] T035 [US2] Add CSS for selectedContext chip in docs/src/components/RAGChatbot/styles.module.css (dismissible chip styling, truncation, tooltip)
- [x] T036 [US2] Add keyboard shortcut documentation (aria-label or help text showing Cmd/Ctrl+K)
- [x] T037 [US2] Test text selection flow per quickstart.md Step 10 (select text, press Cmd/Ctrl+K, verify chatbot opens with context, ask question, verify targeted response)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Conversation History (Priority: P3)

**Goal**: Enable readers to review previous questions/answers and build on previous context without repeating information

**Independent Test**: Ask "What is URDF?", receive answer. Then ask "Can you show an example?" without mentioning URDF. The system should understand from conversation history that "example" refers to URDF and provide relevant examples.

### Implementation for User Story 3

- [x] T038 [US3] Update sendMessage in ChatProvider to include conversation history (extract last 5 messages from session.messages, format as {role, content} array, add to ChatRequest as history field)
- [x] T039 [US3] Update sessionStorage persistence to include panel state (save PANEL_STATE='rag-chatbot-panel-state' key with open/closed boolean)
- [x] T040 [US3] Implement panel state restoration in ChatPanel (load panel state from sessionStorage on mount, restore open/closed state)
- [x] T041 [US3] Implement conversation history restoration (verify ChatProvider loads session from sessionStorage on mount, messages array persists across page navigations)
- [x] T042 [US3] Add scroll-to-bottom behavior in ChatPanel (auto-scroll message list to bottom when new message added, useEffect with messages dependency)
- [x] T043 [US3] Update "New Conversation" button behavior (clear history, generate new sessionId, clear sessionStorage, reset panel state)
- [x] T044 [US3] Test conversation history flow per quickstart.md Step 10 (ask first question, ask follow-up without context, verify backend uses history, navigate to different page, verify history persists)

**Checkpoint**: All user stories 1, 2, and 3 should now be independently functional

---

## Phase 6: User Story 4 - Response Streaming (Priority: P4)

**Goal**: Display AI responses word-by-word as generated for immediate feedback and improved perceived performance

**Independent Test**: Ask a question that generates a long response (e.g., "Explain the ROS 2 navigation stack"). The response should start appearing within 1 second and stream word-by-word rather than appearing all at once after 3-5 seconds.

### Implementation for User Story 4

- [x] T045 [US4] Implement streaming API client in docs/src/components/RAGChatbot/api.ts (streamChatMessage function using fetchEventSource from @microsoft/fetch-event-source, handle POST to /chat/stream)
- [x] T046 [US4] Add SSE event parsing in streamChatMessage (parse event types: tool_call, retrieval, content, done, error, call appropriate callbacks)
- [x] T047 [US4] Update ChatMessage component to support streaming state (add isStreaming prop, show streaming indicator, render incremental content updates)
- [x] T048 [US4] Update sendMessage in ChatProvider to use streaming (if config.streamingEnabled, use streamChatMessage instead of sendChatMessage, create assistant message with isStreaming=true, append content deltas on each token)
- [x] T049 [US4] Implement streaming callbacks in ChatProvider (onToken: append delta to message.content and update state, onDone: finalize message with confidence/sources/metadata and set isStreaming=false, onError: display error message)
- [x] T050 [US4] Add fallback to synchronous in streamChatMessage (catch SSE connection errors, log warning, call sendChatMessage as fallback)
- [x] T051 [US4] Add "Stop" button to ChatInput when streaming (show stop button when isStreaming, onClick abort stream, display partial response)
- [x] T052 [US4] Update auto-scroll behavior for streaming (scroll to bottom on each content delta, not just on message complete)
- [x] T053 [US4] Add CSS for streaming indicator in docs/src/components/RAGChatbot/styles.module.css (typing animation, stop button styling)
- [x] T054 [US4] Test streaming flow per quickstart.md Step 10 (ask question expecting long response, verify first token <1s, verify incremental display, verify final metadata)

**Checkpoint**: All user stories 1, 2, 3, and 4 should now be independently functional

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and ensure production readiness

- [x] T055 [P] Create ConfidenceBadge component in docs/src/components/RAGChatbot/ConfidenceBadge.tsx (display high/medium/low/insufficient with color-coded badge, tooltip explaining confidence meaning)
- [x] T056 [P] Integrate ConfidenceBadge in ChatMessage component (render badge if confidence present, position next to message content)
- [x] T057 [P] Add comprehensive ARIA labels to all components (ChatToggle: aria-label="Open chatbot" aria-expanded, ChatPanel: role="dialog" aria-labelledby, MessageList: role="log" aria-live="polite", ChatInput: aria-label="Type your question")
- [x] T058 [P] Implement keyboard navigation (Esc closes panel, Tab navigates elements, Enter submits message, Shift+Enter inserts newline, focus management on open/close)
- [x] T059 [P] Add focus management (auto-focus input on panel open, return focus to toggle on close, visible focus indicators 2px blue outline)
- [x] T060 [P] Verify color contrast meets WCAG 2.1 AA (text 4.5:1 ratio, interactive elements 3:1 ratio, confidence badges use icons+text not just color)
- [x] T061 [P] Optimize performance with React.memo (memoize ChatMessage component by message.id, memoize expensive callbacks with useCallback)
- [x] T062 [P] Add lazy loading for chatbot in docs/src/theme/Root.tsx (React.lazy import RAGChatbot, wrap in Suspense with fallback=null)
- [x] T063 [P] Implement request validation in API client (validate message 1-1000 chars, validate sessionId UUID v4 format, validate context ≤5000 chars, display user-friendly validation errors)
- [x] T064 [P] Add client-side rate limiting detection (handle 429 responses, respect retry_after header, display "Too many questions. Please wait [N] seconds" message, disable retry button until timer expires)
- [x] T065 [P] Test mobile responsiveness manually (resize viewport to 320px, verify no horizontal scroll, verify full-screen layout, verify touch targets ≥48x48px, test on real iOS/Android devices if available)
- [x] T066 [P] Test accessibility manually (keyboard-only navigation, screen reader announcements, focus indicators, ARIA labels)
- [x] T067 [P] Run quickstart.md validation end-to-end (follow all 11 steps, verify setup time <30 minutes, verify all 5 test cases pass)
- [x] T068 [P] Add error boundary to RAGChatbot component (catch React errors, display fallback UI, log to console in development)
- [x] T069 [P] Add production build configuration check (verify RAG_BACKEND_URL uses https:// in production, warn if http:// in production build)
- [x] T070 [P] Create basic troubleshooting guide in docs/src/components/RAGChatbot/README.md (common issues: CORS errors, backend not responding, streaming not working, session not persisting)
- [x] T071 [P] Verify all success criteria (SC-001 to SC-010) are testable (document how to measure each criterion, e.g., Network tab for latency, Lighthouse for load time)

**Checkpoint**: All user stories complete, polished, and production-ready

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion (T001-T007) - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion (T008-T012)
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3 → P4)
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2, T008-T012) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2, T008-T012) - Integrates with US1 (ChatProvider, ChatInput) but is independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2, T008-T012) - Enhances US1 (ChatProvider sessionStorage) but is independently testable
- **User Story 4 (P4)**: Can start after Foundational (Phase 2, T008-T012) - Enhances US1 (API client, ChatProvider sendMessage) but is independently testable with fallback

### Within Each User Story

- **User Story 1**: Components (T013-T020) can be built in parallel, then integration (T021-T029) sequentially
- **User Story 2**: All tasks can proceed sequentially after US1 complete (enhances existing components)
- **User Story 3**: All tasks can proceed sequentially after US1 complete (enhances sessionStorage logic)
- **User Story 4**: API client tasks (T045-T046) first, then ChatProvider updates (T047-T050), then UI tasks (T051-T053)

### Parallel Opportunities

- **Setup Phase**: T003, T004, T005, T007 can run in parallel (different files)
- **Foundational Phase**: T009, T010, T012 can run in parallel (different files)
- **User Story 1**: T013-T020 (all component files) can run in parallel, then T021-T029 sequentially
- **Polish Phase**: T055-T071 (most tasks) can run in parallel (different concerns)
- **Once Foundational completes**: All user stories (US1, US2, US3, US4) can start in parallel if team capacity allows

---

## Parallel Example: User Story 1

```bash
# After Foundational phase (T008-T012) completes, launch all US1 components in parallel:

# Parallel batch 1 - Component files (can all run simultaneously):
T013: "Create ChatProvider component in docs/src/components/RAGChatbot/ChatContext.tsx"
T014: "Create ChatToggle component in docs/src/components/RAGChatbot/ChatToggle.tsx"
T015: "Create ChatPanel component in docs/src/components/RAGChatbot/ChatPanel.tsx"
T016: "Create ChatMessage component in docs/src/components/RAGChatbot/ChatMessage.tsx"
T017: "Create ChatInput component in docs/src/components/RAGChatbot/ChatInput.tsx"
T018: "Create LoadingIndicator component in docs/src/components/RAGChatbot/LoadingIndicator.tsx"
T019: "Create ErrorMessage component in docs/src/components/RAGChatbot/ErrorMessage.tsx"
T020: "Create SourceCitation component in docs/src/components/RAGChatbot/SourceCitation.tsx"

# Sequential batch 2 - Integration tasks (run after batch 1 completes):
T021: "Integrate react-markdown in ChatMessage component"
T022: "Implement sendMessage method in ChatProvider"
T023: "Implement sessionStorage persistence in ChatProvider"
T024: "Create main RAGChatbot component in docs/src/components/RAGChatbot/index.tsx"
T025: "Create or update docs/src/theme/Root.tsx"
T026: "Add responsive CSS for mobile"
T027: "Add error handling CSS"
T028: "Implement clearHistory method in ChatProvider"
T029: "Add New Conversation button to ChatPanel header"
T030: "Test basic Q&A flow per quickstart.md"
```

---

## Parallel Example: All User Stories (After Foundational)

```bash
# If team has 4 developers, after Foundational phase (T008-T012):

Developer A: Start User Story 1 (T013-T030) - MVP
Developer B: Start User Story 2 (T031-T037) - Text selection (needs US1 components as reference)
Developer C: Start User Story 3 (T038-T044) - Conversation history (needs US1 ChatProvider as reference)
Developer D: Start User Story 4 (T045-T054) - Streaming (needs US1 API client as reference)

# Note: US2, US3, US4 can reference US1 code structure but should remain independently testable
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T007)
2. Complete Phase 2: Foundational (T008-T012) - CRITICAL
3. Complete Phase 3: User Story 1 (T013-T030)
4. **STOP and VALIDATE**: Test User Story 1 independently per quickstart.md
5. Deploy/demo MVP if ready

**Time Estimate**: 20-30 hours for MVP (Setup + Foundational + US1)

### Incremental Delivery

1. Complete Setup (T001-T007) + Foundational (T008-T012) → Foundation ready
2. Add User Story 1 (T013-T030) → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 (T031-T037) → Test independently → Deploy/Demo (MVP + Text Selection)
4. Add User Story 3 (T038-T044) → Test independently → Deploy/Demo (MVP + Text Selection + History)
5. Add User Story 4 (T045-T054) → Test independently → Deploy/Demo (All features)
6. Add Polish (T055-T071) → Production-ready
7. Each story adds value without breaking previous stories

**Time Estimate**: 40-60 hours for all user stories + polish

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup (T001-T007) + Foundational (T008-T012) together - ~5-8 hours
2. Once Foundational is done:
   - Developer A: User Story 1 (T013-T030) - ~15-20 hours
   - Developer B: User Story 2 (T031-T037) - ~5-8 hours (waits for US1 ChatInput reference)
   - Developer C: User Story 3 (T038-T044) - ~5-8 hours (waits for US1 ChatProvider reference)
   - Developer D: User Story 4 (T045-T054) - ~8-12 hours (waits for US1 API client reference)
3. Integrate and test all stories together - ~3-5 hours
4. Complete Polish (T055-T071) together - ~5-8 hours
5. Total parallel time: ~25-35 hours (vs 40-60 hours sequential)

---

## Notes

- **[P] tasks**: Different files, no dependencies, can run in parallel
- **[Story] label**: Maps task to specific user story for traceability (US1, US2, US3, US4)
- **Each user story is independently testable**: Per acceptance scenarios in spec.md and independent test descriptions
- **MVP is User Story 1 only**: Delivers core value (ask questions, receive grounded answers with citations)
- **No test tasks included**: Testing performed manually per quickstart.md and acceptance scenarios (no TDD explicitly requested in spec)
- **Commit after each task**: Or logical group of related tasks
- **Stop at any checkpoint**: Validate story independently before proceeding
- **Docusaurus hot reload**: Use `npm run start` in docs/ for live development feedback
- **Backend must be running**: Start backend (003-rag-agent-api) at http://localhost:8000 before testing
- **Environment variables**: Configure in docs/.env.local for local development, production uses build-time env vars
- **Mobile testing**: Resize browser to 320px or test on real devices (iOS Safari, Android Chrome)
- **Accessibility testing**: Use keyboard-only navigation and screen reader (NVDA, JAWS, VoiceOver) for manual validation

---

## Success Criteria Validation

Each user story maps to success criteria from spec.md:

- **User Story 1 (Basic Q&A)**: SC-001 (latency <5s), SC-002 (load <1s), SC-003 (95% error handling), SC-009 (clickable citations)
- **User Story 2 (Text Selection)**: SC-004 (5000 char limit), SC-007 (keyboard accessible)
- **User Story 3 (Conversation History)**: SC-005 (90% persistence across navigations)
- **User Story 4 (Streaming)**: SC-006 (first token <1s)
- **Polish Phase**: SC-007 (keyboard accessible), SC-008 (mobile compatible), SC-010 (onboarding <30 min)

Validate each criterion during corresponding user story checkpoint.

---

## Total Task Count

- **Phase 1 (Setup)**: 7 tasks (T001-T007)
- **Phase 2 (Foundational)**: 5 tasks (T008-T012)
- **Phase 3 (User Story 1)**: 18 tasks (T013-T030)
- **Phase 4 (User Story 2)**: 7 tasks (T031-T037)
- **Phase 5 (User Story 3)**: 7 tasks (T038-T044)
- **Phase 6 (User Story 4)**: 10 tasks (T045-T054)
- **Phase 7 (Polish)**: 17 tasks (T055-T071)

**Total**: 71 tasks

**MVP (US1 only)**: 30 tasks (T001-T030)
**All user stories**: 54 tasks (T001-T054)
**Production-ready**: 71 tasks (T001-T071)
