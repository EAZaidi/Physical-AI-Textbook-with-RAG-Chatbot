# Backend API Contract: RAG Agent API Integration

**Feature**: 004-rag-chatbot-frontend
**Backend Feature**: 003-rag-agent-api
**Created**: 2025-12-17
**API Version**: v1

## Overview

This document defines the API contract between the RAG Chatbot Frontend and the FastAPI backend (003-rag-agent-api). All endpoints use JSON request/response format except for streaming which uses Server-Sent Events (SSE).

**Base URL**: Configurable via environment variable (default: `http://localhost:8000`)

**Authentication**: Optional API key via `X-API-Key` header (MVP: no authentication)

**CORS**: Backend must allow frontend origin (already configured in 003-rag-agent-api)

---

## Endpoints

### 1. POST /chat/run (Synchronous Chat)

**Purpose**: Send a user question and receive a complete response with citations.

**Request**:
```json
POST /chat/run
Content-Type: application/json
X-API-Key: [optional-api-key]

{
  "message": "What is a ROS 2 node?",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "context": "Optional selected text context (max 5000 chars)",
  "history": [
    {"role": "user", "content": "Previous question"},
    {"role": "assistant", "content": "Previous answer"}
  ]
}
```

**Request Schema**:
```typescript
interface ChatRunRequest {
  message: string;          // Required, max 1000 characters
  session_id: string;       // Required, UUID v4 format
  context?: string;         // Optional, max 5000 characters
  history?: Array<{         // Optional, last 5 messages
    role: 'user' | 'assistant';
    content: string;
  }>;
}
```

**Success Response** (200 OK):
```json
{
  "response": "A ROS 2 node is a fundamental building block...",
  "confidence": 0.92,
  "confidence_level": "high",
  "sources": [
    {
      "chunk_text": "ROS 2 nodes are processes that perform computation...",
      "similarity_score": 0.89,
      "chapter": "Chapter 3: ROS 2 Fundamentals",
      "section": "3.2 Nodes and Communication",
      "url": "/docs/ch3-ros2-fundamentals#nodes",
      "chunk_index": 42,
      "page_number": 87
    }
  ],
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "should_answer": true
}
```

**Response Schema**:
```typescript
interface ChatRunResponse {
  response: string;                    // Markdown formatted response
  confidence: number;                  // 0.0 to 1.0
  confidence_level: 'high' | 'medium' | 'low' | 'insufficient';
  sources: Source[];                   // Array of citations
  session_id: string;                  // Echoed from request
  should_answer: boolean;              // Whether agent answered
  refusal_reason?: string;             // Present if should_answer is false
}

interface Source {
  chunk_text: string;
  similarity_score: number;            // 0.0 to 1.0
  chapter: string;
  section: string;
  url?: string;
  chunk_index: number;
  page_number?: number;
}
```

**Low Confidence Response** (200 OK, should_answer: false):
```json
{
  "response": "",
  "confidence": 0.35,
  "confidence_level": "insufficient",
  "sources": [],
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "should_answer": false,
  "refusal_reason": "No relevant content found with sufficient confidence"
}
```

**Error Responses**:

**400 Bad Request** (Validation Error):
```json
{
  "detail": "Validation error: message exceeds maximum length of 1000 characters"
}
```

**429 Too Many Requests** (Rate Limit):
```json
{
  "detail": "Rate limit exceeded. Please try again later.",
  "retry_after": 60
}
```

**503 Service Unavailable** (Backend Error):
```json
{
  "detail": "Service temporarily unavailable. Database connection failed."
}
```

**Frontend Handling** (from FR-005):
- 400: Display validation error to user, do not retry
- 429: Wait `retry_after` seconds, then retry (max 2 retries)
- 503: Display "Service unavailable" message, retry after 5s (max 2 retries)
- Network errors (CORS, connection refused): Display "Connection error", retry after 3s

**Performance Expectations** (from SC-001):
- p95 latency: < 5 seconds (including backend processing)
- Timeout: 30 seconds (frontend aborts request)

---

### 2. POST /chat/stream (Streaming Chat via SSE)

**Purpose**: Send a user question and receive a streaming response with real-time tokens.

**Request**:
```json
POST /chat/stream
Content-Type: application/json
X-API-Key: [optional-api-key]

{
  "message": "Explain the ROS 2 navigation stack",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "context": "Optional selected text context",
  "history": [...]
}
```

**Request Schema**: Same as `/chat/run` (ChatRunRequest)

**Success Response** (200 OK, text/event-stream):

The response is a stream of Server-Sent Events (SSE) in the following format:

```
event: tool_call
data: {"tool_name": "retrieve_context", "arguments": {"query": "ROS 2 navigation stack"}}

event: retrieval
data: {"query": "ROS 2 navigation stack", "results": [{"chunk_text": "The ROS 2 navigation stack...", "similarity_score": 0.91, "chapter": "Chapter 5", "section": "5.3", "chunk_index": 67}]}

event: content
data: {"delta": "The"}

event: content
data: {"delta": " ROS"}

event: content
data: {"delta": " 2"}

event: content
data: {"delta": " navigation"}

...

event: done
data: {"response": "The ROS 2 navigation stack is a collection...", "confidence": 0.88, "confidence_level": "high", "sources": [...], "session_id": "550e8400-e29b-41d4-a716-446655440000"}
```

**Event Types**:

```typescript
type SSEEventType = 'tool_call' | 'retrieval' | 'content' | 'done' | 'error';
```

**Event Schemas**:

**tool_call** (Agent is calling a tool):
```json
{
  "tool_name": "retrieve_context",
  "arguments": {"query": "user question"}
}
```

**retrieval** (Agent retrieved relevant chunks):
```json
{
  "query": "search query",
  "results": [
    {
      "chunk_text": "excerpt from textbook",
      "similarity_score": 0.89,
      "chapter": "Chapter 3",
      "section": "3.2",
      "url": "/docs/ch3#section",
      "chunk_index": 42
    }
  ]
}
```

**content** (Incremental response token):
```json
{
  "delta": "word or phrase"
}
```

**done** (Stream complete with final metadata):
```json
{
  "response": "Complete response text",
  "confidence": 0.88,
  "confidence_level": "high",
  "sources": [...],
  "session_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

**error** (Stream error):
```json
{
  "error": "Backend processing error",
  "code": "RETRIEVAL_FAILED"
}
```

**Error Handling**:
- If stream fails to connect: Fall back to POST /chat/run (synchronous)
- If stream terminates early: Display partial response with error indicator
- If error event received: Display error message, do not retry

**Frontend Implementation** (from Decision 6):
- Use `@microsoft/fetch-event-source` for POST SSE support
- Buffer `content` deltas to build complete response
- Update UI on each `content` event (auto-scroll)
- Process `done` event to finalize message with metadata
- Provide "Stop" button to abort stream

**Performance Expectations** (from SC-006):
- First token latency: < 1 second
- Token streaming rate: ~50-100 tokens/second
- Total response time: < 5 seconds for typical questions

---

### 3. GET /health (Health Check)

**Purpose**: Verify backend is reachable and operational.

**Request**:
```
GET /health
```

**Success Response** (200 OK):
```json
{
  "status": "healthy",
  "version": "1.0.0"
}
```

**Error Response** (503 Service Unavailable):
```json
{
  "status": "unhealthy",
  "reason": "Database connection failed"
}
```

**Frontend Usage**:
- Check health on chatbot initialization (optional)
- Use for connectivity troubleshooting
- Do NOT poll health endpoint repeatedly (no heartbeat needed)

---

## Request Headers

### Required Headers

```
Content-Type: application/json
```

### Optional Headers

```
X-API-Key: your-api-key-here          # If backend requires authentication
Accept: application/json              # For /chat/run
Accept: text/event-stream             # For /chat/stream
```

### CORS Headers (Backend Provides)

Backend must respond with:
```
Access-Control-Allow-Origin: [frontend-origin]
Access-Control-Allow-Methods: POST, GET, OPTIONS
Access-Control-Allow-Headers: Content-Type, X-API-Key
Access-Control-Max-Age: 86400
```

---

## Error Response Format

All error responses follow this schema:

```typescript
interface ErrorResponse {
  detail: string;              // User-friendly error message
  code?: string;               // Machine-readable error code
  retry_after?: number;        // Seconds to wait before retry (429 only)
}
```

**Common Error Codes**:
- `VALIDATION_ERROR` (400): Invalid request format or content
- `RATE_LIMIT_EXCEEDED` (429): Too many requests
- `SERVICE_UNAVAILABLE` (503): Backend service is down
- `DATABASE_ERROR` (503): Database connection failed
- `RETRIEVAL_FAILED` (500): Vector search failed
- `OPENAI_ERROR` (502): OpenAI API error

---

## Request Validation Rules

### Message Validation
- **Length**: 1 to 1000 characters
- **Content**: Non-empty, trimmed
- **Format**: Plain text (no HTML/scripts)

### Session ID Validation
- **Format**: UUID v4 (8-4-4-4-12 hex digits)
- **Example**: `550e8400-e29b-41d4-a716-446655440000`
- **Generation**: Client-generated via `crypto.randomUUID()`

### Context Validation
- **Length**: 0 to 5000 characters
- **Content**: Plain text (no HTML/scripts)
- **Usage**: Optional, sent when user selects text on page

### History Validation
- **Length**: 0 to 10 messages (backend may limit further)
- **Format**: Array of {role, content} objects
- **Ordering**: Chronological (oldest to newest)

---

## Rate Limiting

**Backend Limits** (from 003-rag-agent-api):
- Per IP: 60 requests/minute
- Per Session: 20 requests/minute

**Frontend Strategy**:
- Do NOT implement client-side rate limiting
- Handle 429 responses with exponential backoff
- Display user-friendly "Please wait" message on rate limit
- Respect `retry_after` header value

---

## Streaming Protocol Details

### SSE Format

Each event follows this structure:
```
event: <event-type>
data: <json-payload>

```

**Important**: Each event ends with two newlines (`\n\n`).

### Parsing Example (TypeScript)

```typescript
import { fetchEventSource } from '@microsoft/fetch-event-source';

await fetchEventSource('http://localhost:8000/chat/stream', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
  },
  body: JSON.stringify({
    message: 'What is URDF?',
    session_id: sessionId,
  }),
  onopen: async (response) => {
    if (response.ok && response.headers.get('content-type') === 'text/event-stream') {
      console.log('Stream opened');
    } else {
      throw new Error('Failed to open stream');
    }
  },
  onmessage: (event) => {
    const eventType = event.event; // 'tool_call' | 'retrieval' | 'content' | 'done' | 'error'
    const data = JSON.parse(event.data);

    switch (eventType) {
      case 'content':
        appendToken(data.delta); // Append to UI
        break;
      case 'done':
        finalizeMessage(data); // Save final metadata
        break;
      case 'error':
        handleError(data.error);
        break;
    }
  },
  onerror: (error) => {
    console.error('Stream error:', error);
    fallbackToSync(); // Use POST /chat/run instead
  },
});
```

### Fallback Strategy

If SSE streaming fails (network error, unsupported browser, CORS issue):
1. Catch error in `onerror` handler
2. Log warning to console
3. Retry request using POST /chat/run (synchronous)
4. Display complete response after it arrives

---

## Backend Behavior Guarantees

From 003-rag-agent-api specification:

1. **Idempotency**: Requests with same `session_id` maintain conversation context
2. **Timeout**: Backend times out after 30 seconds (returns 504)
3. **Confidence Thresholds**:
   - High: confidence ≥ 0.8
   - Medium: 0.6 ≤ confidence < 0.8
   - Low: 0.4 ≤ confidence < 0.6
   - Insufficient: confidence < 0.4 (should_answer: false)
4. **Source Ranking**: Sources ordered by similarity_score (descending)
5. **Session Persistence**: Backend saves conversation to PostgreSQL for future reference

---

## Testing Contract Compliance

### Manual Testing Checklist

**Synchronous Endpoint** (POST /chat/run):
- [ ] Valid request returns 200 with response, confidence, sources
- [ ] Low confidence returns should_answer: false
- [ ] Invalid message (empty, too long) returns 400
- [ ] Missing session_id returns 400
- [ ] Rate limit returns 429 with retry_after
- [ ] Backend error returns 503

**Streaming Endpoint** (POST /chat/stream):
- [ ] Valid request opens SSE stream
- [ ] Stream emits tool_call, retrieval, content, done events in order
- [ ] Content deltas are incremental (no duplicates)
- [ ] Done event includes complete response and metadata
- [ ] Error event terminates stream immediately
- [ ] Aborted stream (client disconnect) does not crash backend

**Health Endpoint** (GET /health):
- [ ] Returns 200 with status: healthy when operational
- [ ] Returns 503 when backend is unhealthy

### Automated Integration Tests

See `specs/004-rag-chatbot-frontend/test-plan.md` for full test suite (to be created).

Example test cases:
1. Send valid question, assert response structure matches schema
2. Send question with context, assert context influences response
3. Simulate network timeout, assert frontend handles gracefully
4. Send 100 requests rapidly, assert 429 handling works
5. Stream response, assert tokens arrive incrementally
6. Abort stream mid-response, assert no errors
7. Send malformed JSON, assert 400 response

---

## Configuration

### Environment Variables

Frontend must configure:
```typescript
const BACKEND_CONFIG = {
  baseUrl: process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000',
  apiKey: process.env.REACT_APP_API_KEY || undefined,
  timeoutMs: 30000,
  retryAttempts: 2,
};
```

### Docusaurus Configuration

In `docusaurus.config.js`:
```javascript
module.exports = {
  customFields: {
    ragBackendUrl: process.env.RAG_BACKEND_URL || 'http://localhost:8000',
    ragApiKey: process.env.RAG_API_KEY,
  },
};
```

Access in components:
```typescript
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const { siteConfig } = useDocusaurusContext();
const backendUrl = siteConfig.customFields.ragBackendUrl;
```

---

## Security Considerations

1. **Input Sanitization**: Backend sanitizes all user input (no XSS risk)
2. **API Key**: Store in environment variable, NEVER commit to git
3. **CORS**: Backend restricts origins (frontend must be allowlisted)
4. **Rate Limiting**: Backend enforces limits (no client-side bypass)
5. **HTTPS**: Production deployments MUST use HTTPS for API calls
6. **No PII**: Do not send personally identifiable information in messages

---

## Version Compatibility

**Current Version**: v1 (2025-12-17)

**Breaking Changes Policy**:
- API version increments on breaking changes
- Frontend must specify `Accept: application/vnd.api.v1+json` for version pinning
- Backend supports previous major version for 6 months

**Non-Breaking Changes** (safe to deploy independently):
- Adding new optional fields to request/response
- Adding new event types to streaming
- Adding new error codes

---

## Support & Troubleshooting

**Common Issues**:

1. **CORS errors**: Verify backend allows frontend origin
2. **Timeout errors**: Check backend health, verify network connectivity
3. **Empty responses**: Backend may have low confidence (check should_answer field)
4. **Streaming not working**: Check browser compatibility, fall back to sync
5. **429 rate limit**: Wait and retry, respect retry_after header

**Debug Logging**:
Enable debug mode in frontend:
```typescript
const DEBUG = process.env.NODE_ENV === 'development';
if (DEBUG) {
  console.log('[RAG] Request:', request);
  console.log('[RAG] Response:', response);
}
```

---

**Next**: See `quickstart.md` for step-by-step integration guide.
