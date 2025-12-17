# Data Model: RAG Chatbot Frontend Integration

**Feature**: 004-rag-chatbot-frontend
**Created**: 2025-12-17
**Status**: Planning

## Overview

This document defines the TypeScript interfaces and data structures for the RAG Chatbot Frontend Integration. All entities are client-side models that facilitate interaction with the backend API (003-rag-agent-api) and manage local state.

## Core Entities

### ChatMessage

Represents a single message in the conversation thread.

```typescript
interface ChatMessage {
  /** Unique identifier for the message (client-generated UUID) */
  id: string;

  /** Role of the message sender */
  role: 'user' | 'assistant';

  /** Message content (plain text for user, markdown for assistant) */
  content: string;

  /** Timestamp when message was created (ISO 8601) */
  timestamp: string;

  /** Optional confidence score from backend (0.0 to 1.0) */
  confidence?: number;

  /** Optional confidence level categorization */
  confidenceLevel?: 'high' | 'medium' | 'low' | 'insufficient';

  /** Optional source citations from backend */
  sources?: Source[];

  /** Optional error message if request failed */
  error?: string;

  /** Optional flag indicating message is still streaming */
  isStreaming?: boolean;
}
```

**Validation Rules** (from FR-002, FR-003, FR-011):
- `role` must be either 'user' or 'assistant'
- `content` must not be empty for user messages
- `timestamp` must be valid ISO 8601 format
- `confidence` must be between 0.0 and 1.0 if present
- `sources` array must contain valid Source objects if present

**State Transitions**:
- User message: Created → Sent → (Success | Error)
- Assistant message: Receiving → Streaming → Complete
- Streaming message: `isStreaming: true` → `isStreaming: false` when done

---

### ChatSession

Represents a conversation thread with multiple messages.

```typescript
interface ChatSession {
  /** Unique session identifier (UUID v4) */
  sessionId: string;

  /** Array of messages in chronological order */
  messages: ChatMessage[];

  /** Timestamp when session was created (ISO 8601) */
  createdAt: string;

  /** Timestamp of last message (ISO 8601) */
  lastUpdatedAt: string;

  /** Optional selected text context for current interaction */
  selectedContext?: string | null;

  /** Current loading state */
  isLoading: boolean;

  /** Current error state */
  error: string | null;
}
```

**Validation Rules** (from FR-007, FR-008, FR-009):
- `sessionId` must be valid UUID v4 format
- `messages` array maintains chronological order (ascending by timestamp)
- `createdAt` and `lastUpdatedAt` must be valid ISO 8601 timestamps
- `selectedContext` limited to 5000 characters (from SC-004)
- Session persisted in sessionStorage across page navigations
- New session generated on page load if none exists or on "New Conversation" action

**Lifecycle**:
1. **Creation**: Generate new UUID, initialize empty messages array
2. **Active**: Add user/assistant messages, update lastUpdatedAt
3. **Persistence**: Save to sessionStorage on every message update
4. **Restoration**: Load from sessionStorage on component mount
5. **Reset**: Generate new sessionId, clear messages array

---

### Source

Represents a citation/reference to textbook content retrieved by the backend.

```typescript
interface Source {
  /** The actual text chunk from the textbook */
  chunkText: string;

  /** Semantic similarity score (0.0 to 1.0) */
  similarityScore: number;

  /** Chapter name where content was found */
  chapter: string;

  /** Section name within the chapter */
  section: string;

  /** Optional URL to the source content */
  url?: string;

  /** Chunk index in the original document */
  chunkIndex: number;

  /** Optional page number if available */
  pageNumber?: number;
}
```

**Validation Rules** (from FR-003, SC-009):
- `chunkText` must not be empty
- `similarityScore` must be between 0.0 and 1.0
- `chapter` and `section` must not be empty
- `url` must be valid HTTP/HTTPS URL if present
- `chunkIndex` must be non-negative integer
- Sources must be clickable and link to source chapter/section (SC-009)

**Display Requirements**:
- Show chapter and section prominently
- Display similarity score as percentage or confidence indicator
- Make source clickable (navigate to url or construct URL from chapter/section)
- Show snippet of chunkText (truncate if needed)

---

### BackendConfig

Configuration for API communication with the FastAPI backend.

```typescript
interface BackendConfig {
  /** Base URL of the backend API (e.g., http://localhost:8000) */
  baseUrl: string;

  /** Request timeout in milliseconds */
  timeoutMs: number;

  /** Number of retry attempts for failed requests */
  retryAttempts: number;

  /** Optional API key for authentication */
  apiKey?: string;

  /** Enable/disable streaming responses */
  streamingEnabled: boolean;

  /** Optional custom headers */
  headers?: Record<string, string>;
}
```

**Validation Rules** (from FR-015, Assumption 4):
- `baseUrl` must be valid HTTP/HTTPS URL
- `timeoutMs` must be positive integer (default: 30000ms per SC-001)
- `retryAttempts` must be non-negative integer (default: 2)
- `apiKey` required if backend enforces authentication
- `streamingEnabled` defaults to true, falls back to sync if SSE fails

**Configuration Sources** (from Decision 7):
- Environment variables via Docusaurus customFields
- Runtime configuration via `docusaurus.config.js`
- Local override via `.env.local` for development

**Default Values**:
```typescript
const DEFAULT_BACKEND_CONFIG: BackendConfig = {
  baseUrl: 'http://localhost:8000',
  timeoutMs: 30000,
  retryAttempts: 2,
  streamingEnabled: true,
  headers: {
    'Content-Type': 'application/json',
  },
};
```

---

## Supporting Types

### ChatRequest

Request payload for backend API calls.

```typescript
interface ChatRequest {
  /** User's question or message */
  message: string;

  /** Session identifier for conversation continuity */
  sessionId: string;

  /** Optional selected text context */
  context?: string;

  /** Optional conversation history for multi-turn context */
  history?: Array<{role: 'user' | 'assistant'; content: string}>;
}
```

**Validation Rules** (from FR-001, FR-006):
- `message` must not be empty and max 1000 characters
- `sessionId` must be valid UUID v4
- `context` limited to 5000 characters (from SC-004)
- `history` includes last N messages (configurable, default: 5)

---

### ChatResponse

Response payload from backend API (synchronous endpoint).

```typescript
interface ChatResponse {
  /** Generated response text (markdown formatted) */
  response: string;

  /** Overall confidence score (0.0 to 1.0) */
  confidence: number;

  /** Confidence level categorization */
  confidenceLevel: 'high' | 'medium' | 'low' | 'insufficient';

  /** Source citations */
  sources: Source[];

  /** Session identifier (echoed back) */
  sessionId: string;

  /** Whether the agent decided to answer */
  shouldAnswer: boolean;

  /** Optional refusal reason if shouldAnswer is false */
  refusalReason?: string;
}
```

**Validation Rules** (from Backend API spec 003-rag-agent-api):
- `response` is empty if shouldAnswer is false
- `confidence` between 0.0 and 1.0
- `confidenceLevel` must match confidence thresholds
- `sources` array may be empty if no relevant content found
- `shouldAnswer` is false when confidence < threshold or out-of-domain

---

### StreamEvent

SSE event types for streaming responses.

```typescript
type StreamEventType = 'tool_call' | 'retrieval' | 'content' | 'done' | 'error';

interface StreamEvent {
  /** Event type */
  event: StreamEventType;

  /** Event payload (varies by event type) */
  data: StreamEventData;
}

type StreamEventData =
  | ToolCallData
  | RetrievalData
  | ContentData
  | DoneData
  | ErrorData;

interface ToolCallData {
  toolName: string;
  arguments: Record<string, any>;
}

interface RetrievalData {
  query: string;
  results: Source[];
}

interface ContentData {
  delta: string; // Incremental text token
}

interface DoneData {
  response: string; // Complete response
  confidence: number;
  confidenceLevel: string;
  sources: Source[];
  sessionId: string;
}

interface ErrorData {
  error: string;
  code?: string;
}
```

**Validation Rules** (from FR-010, Decision 6):
- Events arrive in order: tool_call → retrieval → content (N times) → done | error
- Frontend buffers `content` deltas to build complete response
- `done` event includes final metadata (confidence, sources)
- `error` event terminates stream immediately

---

## State Management

### ChatContext (React Context)

Global state for chatbot functionality.

```typescript
interface ChatContextType {
  /** Current chat session */
  session: ChatSession;

  /** Send a message and receive response */
  sendMessage: (message: string) => Promise<void>;

  /** Clear conversation history and start new session */
  clearHistory: () => void;

  /** Set selected text context */
  setSelectedContext: (text: string | null) => void;

  /** Backend configuration */
  config: BackendConfig;
}
```

**Context Provider Responsibilities** (from Decision 2):
- Initialize session from sessionStorage or create new
- Persist session to sessionStorage on every update
- Handle message sending (sync or stream based on config)
- Manage loading and error states
- Provide text selection context
- Handle retry logic for failed requests

---

## Storage Schema

### SessionStorage Keys

```typescript
const STORAGE_KEYS = {
  /** Current chat session */
  CHAT_SESSION: 'rag-chatbot-session',

  /** Chat panel open/closed state */
  PANEL_STATE: 'rag-chatbot-panel-state',

  /** User preferences (optional) */
  PREFERENCES: 'rag-chatbot-preferences',
} as const;
```

**Persistence Rules** (from FR-014, SC-005):
- `CHAT_SESSION` saved on every message update (90% persistence target)
- `PANEL_STATE` saved when user toggles chat panel
- Data cleared when browser session ends (sessionStorage, not localStorage)
- No cross-device or cross-browser persistence

---

## Validation & Error Handling

### Error Types

```typescript
interface ChatError {
  /** Error type classification */
  type: 'network' | 'validation' | 'backend' | 'timeout' | 'unknown';

  /** User-friendly error message */
  message: string;

  /** Optional technical details for debugging */
  details?: string;

  /** HTTP status code if applicable */
  statusCode?: number;

  /** Whether error is retryable */
  retryable: boolean;
}
```

**Error Mapping** (from FR-005):
- Network errors (CORS, DNS, connection refused) → `type: 'network'`, retryable: true
- 400 validation errors → `type: 'validation'`, retryable: false
- 429 rate limit → `type: 'backend'`, retryable: true (with delay)
- 503 service unavailable → `type: 'backend'`, retryable: true
- Timeout (>30s) → `type: 'timeout'`, retryable: true

**User-Friendly Messages**:
- Network: "Unable to connect to the chatbot service. Please check your connection."
- Validation: "Your question is too long. Please try a shorter question."
- Rate Limit: "Too many questions. Please wait a moment before trying again."
- Service Unavailable: "The chatbot is temporarily unavailable. Please try again."
- Timeout: "The response is taking longer than expected. Please try again."

---

## Relationships & Dependencies

### Entity Relationships

```
ChatSession (1) ──> (N) ChatMessage
ChatMessage (1) ──> (N) Source
ChatContext (1) ──> (1) ChatSession
ChatContext (1) ──> (1) BackendConfig
```

**Key Dependencies**:
- ChatMessage depends on Source for citations (optional)
- ChatSession aggregates ChatMessage instances
- ChatContext manages ChatSession lifecycle
- BackendConfig determines API behavior (sync vs stream)

### External Dependencies

**From Backend API (003-rag-agent-api)**:
- POST /chat/run → ChatResponse
- POST /chat/stream → StreamEvent[]
- GET /health → {status: string}

**From Browser APIs**:
- window.getSelection() → Text selection capture
- window.sessionStorage → Session persistence
- EventSource / fetch-event-source → SSE streaming
- crypto.randomUUID() → Session ID generation

**From Libraries**:
- react-markdown → Render assistant responses
- @microsoft/fetch-event-source → POST SSE support

---

## Constitution Compliance

**Principle 1 - Feature Quality** (SC-001 to SC-010):
- All entities support measurable success criteria
- Performance targets: <5s response (SC-001), <1s interface load (SC-002)
- Error handling: 95% graceful handling (SC-003)

**Principle 2 - Code Quality**:
- TypeScript interfaces provide type safety
- Validation rules prevent invalid states
- Clear entity relationships and boundaries

**Principle 3 - Testing**:
- All entities have testable validation rules
- State transitions are clearly defined
- Error types support comprehensive error handling tests

**Principle 4 - Performance**:
- Streaming support reduces perceived latency (SC-006)
- SessionStorage provides fast persistence
- Minimal data structures (no unnecessary nesting)

**Principle 5 - Security**:
- No sensitive data in client-side storage
- Optional API key support (BackendConfig)
- Input validation (max lengths, format checks)

**Principle 6 - Architecture**:
- Clear separation: data models, state management, API client
- Context API provides centralized state
- Loose coupling via interfaces

**Principle 7 - Documentation**:
- All entities fully documented with purpose, validation, lifecycle
- Examples provided for complex types
- Clear relationships and dependencies

---

## Notes & Assumptions

1. **Client-Side Only**: All entities are client-side TypeScript interfaces. No backend database models.

2. **Session Scope**: Sessions are browser-session-scoped (sessionStorage), not persistent across browser restarts.

3. **Message History**: Frontend stores complete message history in memory and sessionStorage. Backend API (003-rag-agent-api) manages its own conversation persistence.

4. **Confidence Thresholds**: Frontend displays confidence levels returned by backend. No client-side threshold logic.

5. **Source URLs**: Backend may not provide direct URLs. Frontend may need to construct URLs from chapter/section metadata.

6. **Streaming Fallback**: If streaming fails, frontend falls back to synchronous POST /chat/run endpoint.

7. **Mobile Optimization**: All entities designed for mobile (no large embedded resources, efficient serialization).

8. **Accessibility**: ChatMessage and UI state support screen reader announcements (role-based message identification).

---

**Next Steps**: Create API contracts in `contracts/` directory and quickstart guide in `quickstart.md`.
