# Research & Technical Decisions: RAG Chatbot Frontend Integration

**Feature**: 004-rag-chatbot-frontend
**Phase**: 0 (Research & Discovery)
**Date**: 2025-12-17

## Overview

This document captures research findings and technical decisions for integrating the RAG chatbot frontend with the existing backend API. The goal is to add an interactive chat interface to the Docusaurus-based textbook that communicates with the FastAPI backend (003-rag-agent-api).

---

## Decision 1: Frontend Framework & Integration Approach

### Context
Need to add a chat UI component to the existing Docusaurus site (React-based static site generator).

### Options Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **A. Docusaurus Plugin/Component** | Native integration, access to React context, matches site styling | Requires understanding Docusaurus plugin system | **✅ SELECTED** |
| B. Standalone Widget (iframe) | Complete isolation, easy to develop separately | Poor UX (styling mismatch), limited access to page content, CORS complexities | ❌ Rejected |
| C. Browser Extension | Zero changes to frontend codebase | Requires users to install extension, not discoverable | ❌ Rejected |

### Decision
**Use Docusaurus React Components** integrated directly into the site

### Rationale
- Docusaurus is built on React - we can create custom React components
- Full access to page DOM for text selection feature
- Consistent styling with site theme
- No additional infrastructure needed
- SEO-friendly (no iframe isolation issues)

### Implementation Approach
- Create React component in `docs/src/components/ChatWidget/`
- Use Docusaurus theme system for styling consistency
- Inject component via `@theme/Root` swizzling (Docusaurus pattern)
- Component persists across page navigations

### References
- Docusaurus Client API: https://docusaurus.io/docs/advanced/client
- Swizzling: https://docusaurus.io/docs/swizzling
- React Context in Docusaurus: https://docusaurus.io/docs/api/themes/configuration

---

## Decision 2: State Management Strategy

### Context
Need to manage chat state (messages, session_id, loading status) across component lifecycle and page navigations.

### Options Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **A. React Context + sessionStorage** | Simple, matches Docusaurus patterns, persists across navigation | Manual serialization needed | **✅ SELECTED** |
| B. Redux/Zustand | Robust, devtools, middleware | Overkill for simple chat state, adds dependency | ❌ Rejected |
| C. Component State Only | Simplest | Lost on navigation, no persistence | ❌ Rejected |

### Decision
**React Context API + sessionStorage for persistence**

### Rationale
- React Context provides global state access without prop drilling
- sessionStorage persists state across page navigation (but not browser sessions)
- Matches requirements (SC-005: 90% history persistence across navigations)
- No external dependencies needed
- Easy to test and reason about

### Implementation Details

```typescript
// ChatContext structure
interface ChatContextType {
  messages: ChatMessage[];
  sessionId: string;
  isLoading: boolean;
  error: string | null;
  selectedContext: string | null;
  sendMessage: (message: string) => Promise<void>;
  clearHistory: () => void;
  setSelectedContext: (text: string | null) => void;
}
```

**Persistence Strategy**:
- Save to sessionStorage on every state change (messages, sessionId)
- Load from sessionStorage on component mount
- Clear on "New Conversation" action

---

## Decision 3: API Client Architecture

### Context
Need HTTP client to communicate with FastAPI backend (POST /chat/run, POST /chat/stream).

### Options Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **A. Native fetch + custom hooks** | Zero dependencies, modern API, supports streaming | Manual error handling, retry logic | **✅ SELECTED** |
| B. axios | Popular, interceptors, automatic retries | No native SSE support, adds 13KB | ❌ Rejected |
| C. SWR/React Query | Caching, automatic retries, devtools | Designed for GET requests, chat doesn't fit pattern | ❌ Rejected |

### Decision
**Native fetch API with custom React hooks**

### Rationale
- Zero dependencies (fetch is built-in)
- Native SSE support via EventSource API
- Streaming support via `fetch` + ReadableStream
- Custom hooks encapsulate API logic cleanly
- Full control over retry logic and error handling

### Implementation Approach

**Custom Hooks**:
```typescript
// useChatAPI.ts
export function useChatAPI() {
  const sendMessage = async (message: string, sessionId: string) => {
    // POST /chat/run with fetch
  };

  const sendMessageStream = (message: string, sessionId: string) => {
    // POST /chat/stream with EventSource
  };

  return { sendMessage, sendMessageStream };
}

// useTextSelection.ts
export function useTextSelection() {
  // Listen to 'mouseup' events, extract window.getSelection()
}
```

**Error Handling**:
- Retry 503 errors (backend unavailable) with exponential backoff (3 attempts)
- Display user-friendly messages for 429 (rate limit), 400 (validation)
- Log errors to console for debugging (development mode)

---

## Decision 4: Markdown Rendering Library

### Context
Backend responses are plain text but may include markdown formatting (code blocks, lists, bold).

### Options Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **A. react-markdown** | React-native, secure (sanitizes HTML), popular (12M downloads/week) | 16KB gzipped | **✅ SELECTED** |
| B. marked.js | Smallest (6KB), fast | Not React-specific, manual sanitization needed | ❌ Rejected |
| C. remark + rehype | Most powerful, extensible | Complex setup, overkill for basic markdown | ❌ Rejected |

### Decision
**react-markdown** (v9+)

### Rationale
- React-first library (renders to React components, not HTML strings)
- Built-in XSS protection (sanitizes HTML by default)
- Syntax highlighting via `react-syntax-highlighter` plugin
- Widely used in documentation sites
- Good performance for chat-length content

### Features Needed
- Code block syntax highlighting (Python, YAML, JSON)
- Lists (ordered, unordered)
- Bold, italic, inline code
- Links (auto-detect URLs)

### Package
```json
{
  "react-markdown": "^9.0.0",
  "remark-gfm": "^4.0.0", // GitHub Flavored Markdown (tables, strikethrough)
  "react-syntax-highlighter": "^15.5.0" // Code highlighting
}
```

---

## Decision 5: Text Selection Implementation

### Context
Need to capture text selected by user on page (FR-006) and send as context to backend.

### Options Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **A. window.getSelection() + keyboard shortcut** | Standard API, accessible | Requires keyboard listener management | **✅ SELECTED** |
| B. Context menu (right-click) | Discoverable, standard pattern | Docusaurus may override context menu | ⚠️ Backup option |
| C. Floating button on selection | Modern, used by Medium/Notion | More complex UI, may conflict with page layout | ❌ Rejected |

### Decision
**window.getSelection() API + keyboard shortcut (Cmd/Ctrl+K)**

### Rationale
- `window.getSelection()` is the standard browser API for text selection
- Keyboard shortcut (Cmd+K) is common for "quick actions" (VS Code, Slack, Linear)
- Works consistently across all pages without modifying Docusaurus
- Accessible (keyboard-driven workflow)

### Implementation Details

```typescript
// Text selection handler
document.addEventListener('keydown', (e) => {
  if ((e.metaKey || e.ctrlKey) && e.key === 'k') {
    e.preventDefault();
    const selectedText = window.getSelection()?.toString();
    if (selectedText) {
      openChatWithContext(selectedText);
    } else {
      openChat(); // No selection, open normally
    }
  }
});
```

**User Flow**:
1. Reader selects text on page
2. Press Cmd+K (Mac) or Ctrl+K (Windows/Linux)
3. Chat panel opens with selected text shown as context box
4. Reader types question → both question + context sent to backend

---

## Decision 6: Streaming Implementation (SSE vs Fetch Streaming)

### Context
Backend supports streaming via POST /chat/stream (Server-Sent Events). Need to decide frontend implementation.

### Options Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **A. EventSource API (SSE)** | Built-in, automatic reconnection, structured events | POST not supported natively (need workaround) | **✅ SELECTED** |
| B. fetch + ReadableStream | Full control, supports POST | Manual parsing, no reconnection | ❌ Rejected |
| C. WebSockets | Bidirectional, real-time | Overkill (no server->client push needed), backend doesn't support | ❌ Rejected |

### Decision
**EventSource API with polyfill for POST requests**

### Rationale
- Backend uses SSE format (event: content, event: done, event: error)
- EventSource handles reconnection automatically
- Structured event parsing (no manual line splitting)
- Use `fetch-event-source` polyfill for POST support

### Implementation Approach

**Package**: `@microsoft/fetch-event-source` (10KB, supports POST SSE)

```typescript
import { fetchEventSource } from '@microsoft/fetch-event-source';

await fetchEventSource('/chat/stream', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({ message, session_id, stream: true }),
  onmessage(event) {
    if (event.event === 'content') {
      appendToken(JSON.parse(event.data).delta);
    } else if (event.event === 'done') {
      finalize(JSON.parse(event.data));
    } else if (event.event === 'error') {
      handleError(JSON.parse(event.data));
    }
  },
  onerror(err) {
    console.error('Stream error:', err);
    throw err; // Abort streaming
  }
});
```

**Fallback**: If streaming fails, fall back to synchronous POST /chat/run

---

## Decision 7: Configuration Management (Backend URL)

### Context
Need to configure backend API URL (http://localhost:8000 for dev, https://api.example.com for production).

### Options Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **A. Environment variables (.env + Docusaurus config)** | Standard pattern, build-time configuration | Requires rebuild for URL change | **✅ SELECTED** |
| B. Runtime config file (config.json) | Can change without rebuild | Extra HTTP request, security concerns | ❌ Rejected |
| C. Hardcoded with conditional logic | No setup needed | Not configurable, brittle | ❌ Rejected |

### Decision
**Environment variables via Docusaurus customFields**

### Rationale
- Docusaurus supports env vars via `customFields` in docusaurus.config.js
- Build-time configuration is secure (no exposed API URLs in runtime config)
- Supports multiple environments (dev, staging, production)
- Standard practice for static sites

### Implementation

**docusaurus.config.js**:
```javascript
module.exports = {
  customFields: {
    RAG_API_URL: process.env.RAG_API_URL || 'http://localhost:8000',
    RAG_API_TIMEOUT: process.env.RAG_API_TIMEOUT || '30000', // 30 seconds
  },
};
```

**Access in React**:
```typescript
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const { siteConfig } = useDocusaurusContext();
const apiUrl = siteConfig.customFields.RAG_API_URL;
```

**.env.development**:
```env
RAG_API_URL=http://localhost:8000
```

**.env.production**:
```env
RAG_API_URL=https://api.yoursite.com
```

---

## Decision 8: CORS Handling

### Context
Frontend (https://your-site.com) needs to call backend API (http://localhost:8000 or https://api.yoursite.com). CORS must be configured on backend.

### Resolution
**Backend CORS configuration is already handled** (003-rag-agent-api has CORS middleware configured)

### Verification Checklist
1. ✅ Backend allows origin: `https://your-site.com` (or `*` for development)
2. ✅ Backend allows methods: `POST`, `OPTIONS`
3. ✅ Backend allows headers: `Content-Type`, `X-API-Key` (if using authentication)
4. ✅ Backend exposes headers: `Retry-After` (for 429 errors)

**Frontend Action**: Test CORS during development. If errors occur, verify backend CORS config in `main.py`.

---

## Decision 9: Accessibility (WCAG 2.1 AA Compliance)

### Context
Chat interface must be keyboard-accessible (FR-012) and screen reader friendly (SC-007).

### Requirements from Spec
- Keyboard navigation (Tab, Enter, Esc)
- Screen reader support (ARIA labels)
- Focus management (modal behavior)

### Implementation Strategy

**Keyboard Shortcuts**:
- `Cmd/Ctrl+K`: Open chat (with or without selection)
- `Esc`: Close chat
- `Enter`: Submit message
- `Shift+Enter`: New line in input
- `Tab`: Navigate between input, buttons, history

**ARIA Labels**:
```jsx
<div role="dialog" aria-labelledby="chat-title" aria-modal="true">
  <h2 id="chat-title">AI Chatbot</h2>
  <textarea aria-label="Ask a question" aria-describedby="chat-hint" />
  <span id="chat-hint">Press Enter to send, Shift+Enter for new line</span>
  <button aria-label="Send message">Send</button>
</div>
```

**Focus Management**:
- Trap focus within chat modal when open
- Restore focus to trigger element on close
- Auto-focus input field when chat opens

**Screen Reader Announcements**:
- Announce new messages via `aria-live="polite"` region
- Announce errors via `aria-live="assertive"` region
- Announce loading state ("Searching textbook...")

---

## Decision 10: Mobile Responsiveness

### Context
Chat must work on mobile devices (320px to 768px screens) per FR-013 and SC-008.

### Design Decisions

**Layout Strategy**:
- **Desktop (>768px)**: Sidebar panel (300px wide), slides in from right, overlays content
- **Mobile (<768px)**: Full-screen modal, covers entire viewport, swipe down to close

**Touch Interactions**:
- Tap outside to close (desktop)
- Swipe down to close (mobile)
- Touch-friendly button sizes (44x44px minimum per iOS HIG)

**Responsive Breakpoints**:
```css
/* Mobile: Full screen */
@media (max-width: 768px) {
  .chat-widget {
    width: 100vw;
    height: 100vh;
    position: fixed;
    top: 0;
    left: 0;
  }
}

/* Desktop: Sidebar */
@media (min-width: 769px) {
  .chat-widget {
    width: 400px;
    height: 100vh;
    position: fixed;
    right: 0;
    top: 0;
  }
}
```

---

## Technical Stack Summary

| Category | Technology | Version | Rationale |
|----------|------------|---------|-----------|
| **Frontend Framework** | React (via Docusaurus) | 18.x | Already in use, component-based |
| **State Management** | React Context + sessionStorage | Built-in | Simple, matches requirements |
| **HTTP Client** | fetch API | Built-in | Zero deps, supports streaming |
| **Streaming** | @microsoft/fetch-event-source | 2.0.1 | POST SSE support |
| **Markdown** | react-markdown + remark-gfm | 9.0.0 | Secure, React-native |
| **Syntax Highlighting** | react-syntax-highlighter | 15.5.0 | Code block highlighting |
| **UUID Generation** | crypto.randomUUID() | Built-in | Session ID generation |
| **Styling** | CSS Modules + Docusaurus Theme | Built-in | Theme consistency |

---

## Performance Considerations

### Bundle Size Impact
- react-markdown: ~16KB gzipped
- @microsoft/fetch-event-source: ~10KB gzipped
- react-syntax-highlighter: ~20KB gzipped (lazy-loaded)
- Custom components: ~15KB estimated

**Total**: ~61KB additional bundle size (acceptable for interactive feature)

### Optimization Strategies
1. **Code Splitting**: Lazy-load chat component (only when opened)
2. **Markdown Lazy Load**: Load react-markdown on first message render
3. **Syntax Highlighter Lazy Load**: Load only when code block detected
4. **Image Optimization**: Use SVG for icons (no raster images)

### Expected Performance
- SC-001: p95 latency < 5s ✅ (backend dependent)
- SC-002: Interface load < 1s ✅ (with code splitting)
- SC-006: Streaming first token < 1s ✅ (SSE)

---

## Security Considerations

### XSS Prevention
- ✅ react-markdown sanitizes HTML by default
- ✅ No `dangerouslySetInnerHTML` usage
- ✅ User input sanitized before sending to backend

### CORS
- ✅ Backend CORS configured (003-rag-agent-api)
- ✅ No credentials sent (no cookies, no authentication for MVP)

### Content Security Policy
- ✅ No inline scripts
- ✅ No eval() usage
- ✅ External dependencies loaded from npm (no CDN)

---

## Open Questions / Risks

### Risk 1: Docusaurus Version Compatibility
**Risk**: Docusaurus API changes between versions (currently v3.x)
**Mitigation**: Use stable Docusaurus APIs (useDocusaurusContext, theme system), avoid experimental features

### Risk 2: Backend Availability
**Risk**: Backend offline = chat non-functional
**Mitigation**:
- Clear error messages ("Backend unavailable, please try again later")
- Health check endpoint (/health) to verify backend before opening chat
- Retry logic with exponential backoff

### Risk 3: Mobile Keyboard Overlap
**Risk**: Mobile keyboard may overlap chat input on iOS/Android
**Mitigation**:
- Use `window.visualViewport` API to detect keyboard
- Adjust chat height dynamically when keyboard appears
- Test on iOS Safari, Android Chrome

---

## Next Steps (Phase 1)

1. **Generate data-model.md**: Define TypeScript interfaces for ChatMessage, ChatSession, Source, etc.
2. **Generate contracts/**: Document backend API integration (POST /chat/run, POST /chat/stream)
3. **Generate quickstart.md**: Developer setup guide (install deps, configure backend URL, test locally)
4. **Update agent context**: Add React, Docusaurus, fetch-event-source to technology stack

---

**Research Phase Complete** ✅
All technical unknowns resolved. Ready for Phase 1 (Design & Contracts).
