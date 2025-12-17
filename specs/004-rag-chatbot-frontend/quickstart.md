# Quickstart Guide: RAG Chatbot Frontend Integration

**Feature**: 004-rag-chatbot-frontend
**Created**: 2025-12-17
**Target Time**: < 30 minutes (from SC-010)

## Overview

This guide walks you through integrating the RAG chatbot into the Physical AI textbook frontend. By the end, you'll have a working chatbot that can answer questions about the textbook content.

**Prerequisites**:
- Backend API (003-rag-agent-api) is running at `http://localhost:8000`
- Node.js 18+ and npm/yarn installed
- Basic knowledge of React and TypeScript
- Docusaurus project is set up (confirmed in `docs/` directory)

**Success Criteria**: Ask "What is a ROS 2 node?" and receive a grounded answer with citations.

---

## Step 1: Verify Backend is Running

Before starting frontend development, ensure the backend is operational.

```bash
# Test backend health
curl http://localhost:8000/health

# Expected response:
# {"status": "healthy", "version": "1.0.0"}

# Test chat endpoint
curl -X POST http://localhost:8000/chat/run \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is URDF?",
    "session_id": "550e8400-e29b-41d4-a716-446655440000"
  }'

# Expected response (truncated):
# {
#   "response": "URDF (Unified Robot Description Format) is...",
#   "confidence": 0.89,
#   "sources": [...]
# }
```

If backend is not running, follow backend setup in `backend/README.md`.

---

## Step 2: Configure Environment Variables

Create `.env.local` in the Docusaurus project root:

```bash
# .env.local (for local development)
RAG_BACKEND_URL=http://localhost:8000
RAG_API_KEY=                           # Optional, leave empty if backend has no auth
```

**For production**, set these as build-time environment variables:
```bash
RAG_BACKEND_URL=https://api.example.com
RAG_API_KEY=your-production-api-key
```

---

## Step 3: Update Docusaurus Configuration

Edit `docusaurus.config.js` to expose environment variables:

```javascript
// docusaurus.config.js
module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  // ... other config

  customFields: {
    // Expose RAG backend configuration
    ragBackendUrl: process.env.RAG_BACKEND_URL || 'http://localhost:8000',
    ragApiKey: process.env.RAG_API_KEY || undefined,
  },

  // Enable CORS for local development (optional)
  scripts: [],
};
```

---

## Step 4: Install Dependencies

Install required packages:

```bash
cd docs/  # Navigate to Docusaurus project

# Install chatbot dependencies
npm install --save \
  react-markdown \
  @microsoft/fetch-event-source \
  remark-gfm

# Install dev dependencies
npm install --save-dev \
  @types/react \
  @types/node
```

**Package Purposes**:
- `react-markdown`: Render assistant responses (markdown → HTML)
- `@microsoft/fetch-event-source`: SSE streaming with POST support
- `remark-gfm`: GitHub Flavored Markdown (tables, strikethrough)

---

## Step 5: Create Directory Structure

Create the chatbot component directory structure:

```bash
mkdir -p src/components/RAGChatbot
touch src/components/RAGChatbot/index.tsx
touch src/components/RAGChatbot/ChatContext.tsx
touch src/components/RAGChatbot/ChatPanel.tsx
touch src/components/RAGChatbot/ChatMessage.tsx
touch src/components/RAGChatbot/types.ts
touch src/components/RAGChatbot/api.ts
touch src/components/RAGChatbot/styles.module.css
```

**File Purposes**:
- `index.tsx`: Main chatbot component (exported)
- `ChatContext.tsx`: React Context for state management
- `ChatPanel.tsx`: Chat UI (messages, input, toggle button)
- `ChatMessage.tsx`: Individual message rendering
- `types.ts`: TypeScript interfaces (from data-model.md)
- `api.ts`: API client functions (fetch, streaming)
- `styles.module.css`: Component styles

---

## Step 6: Implement Core Types

Create `src/components/RAGChatbot/types.ts`:

```typescript
// src/components/RAGChatbot/types.ts

export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: string;
  confidence?: number;
  confidenceLevel?: 'high' | 'medium' | 'low' | 'insufficient';
  sources?: Source[];
  error?: string;
  isStreaming?: boolean;
}

export interface Source {
  chunkText: string;
  similarityScore: number;
  chapter: string;
  section: string;
  url?: string;
  chunkIndex: number;
  pageNumber?: number;
}

export interface ChatSession {
  sessionId: string;
  messages: ChatMessage[];
  createdAt: string;
  lastUpdatedAt: string;
  selectedContext?: string | null;
  isLoading: boolean;
  error: string | null;
}

export interface BackendConfig {
  baseUrl: string;
  timeoutMs: number;
  retryAttempts: number;
  apiKey?: string;
  streamingEnabled: boolean;
}

export interface ChatRequest {
  message: string;
  session_id: string;
  context?: string;
  history?: Array<{role: 'user' | 'assistant'; content: string}>;
}

export interface ChatResponse {
  response: string;
  confidence: number;
  confidence_level: 'high' | 'medium' | 'low' | 'insufficient';
  sources: Source[];
  session_id: string;
  should_answer: boolean;
  refusal_reason?: string;
}
```

---

## Step 7: Implement API Client

Create `src/components/RAGChatbot/api.ts`:

```typescript
// src/components/RAGChatbot/api.ts
import { fetchEventSource } from '@microsoft/fetch-event-source';
import type { ChatRequest, ChatResponse, BackendConfig } from './types';

export async function sendChatMessage(
  request: ChatRequest,
  config: BackendConfig
): Promise<ChatResponse> {
  const response = await fetch(`${config.baseUrl}/chat/run`, {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
      ...(config.apiKey ? { 'X-API-Key': config.apiKey } : {}),
    },
    body: JSON.stringify(request),
    signal: AbortSignal.timeout(config.timeoutMs),
  });

  if (!response.ok) {
    const error = await response.json().catch(() => ({ detail: 'Unknown error' }));
    throw new Error(error.detail || `HTTP ${response.status}`);
  }

  return response.json();
}

export async function streamChatMessage(
  request: ChatRequest,
  config: BackendConfig,
  callbacks: {
    onToken: (token: string) => void;
    onDone: (response: ChatResponse) => void;
    onError: (error: Error) => void;
  }
): Promise<void> {
  try {
    await fetchEventSource(`${config.baseUrl}/chat/stream`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        ...(config.apiKey ? { 'X-API-Key': config.apiKey } : {}),
      },
      body: JSON.stringify(request),
      onopen: async (response) => {
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }
      },
      onmessage: (event) => {
        const data = JSON.parse(event.data);
        switch (event.event) {
          case 'content':
            callbacks.onToken(data.delta);
            break;
          case 'done':
            callbacks.onDone(data);
            break;
          case 'error':
            callbacks.onError(new Error(data.error));
            break;
        }
      },
      onerror: (error) => {
        callbacks.onError(error);
        throw error; // Stop retrying
      },
    });
  } catch (error) {
    // Fallback to synchronous if streaming fails
    console.warn('Streaming failed, falling back to sync:', error);
    const response = await sendChatMessage(request, config);
    callbacks.onDone(response);
  }
}
```

---

## Step 8: Implement Chat Context

Create `src/components/RAGChatbot/ChatContext.tsx`:

```typescript
// src/components/RAGChatbot/ChatContext.tsx
import React, { createContext, useContext, useState, useEffect } from 'react';
import type { ChatSession, ChatMessage, BackendConfig } from './types';
import { sendChatMessage, streamChatMessage } from './api';

interface ChatContextType {
  session: ChatSession;
  sendMessage: (message: string) => Promise<void>;
  clearHistory: () => void;
  setSelectedContext: (text: string | null) => void;
  config: BackendConfig;
}

const ChatContext = createContext<ChatContextType | undefined>(undefined);

const STORAGE_KEY = 'rag-chatbot-session';

function generateSessionId(): string {
  return crypto.randomUUID();
}

function loadSession(): ChatSession | null {
  try {
    const stored = sessionStorage.getItem(STORAGE_KEY);
    return stored ? JSON.parse(stored) : null;
  } catch {
    return null;
  }
}

function saveSession(session: ChatSession): void {
  sessionStorage.setItem(STORAGE_KEY, JSON.stringify(session));
}

export function ChatProvider({ children, config }: { children: React.ReactNode; config: BackendConfig }) {
  const [session, setSession] = useState<ChatSession>(() => {
    const loaded = loadSession();
    if (loaded) return loaded;

    return {
      sessionId: generateSessionId(),
      messages: [],
      createdAt: new Date().toISOString(),
      lastUpdatedAt: new Date().toISOString(),
      selectedContext: null,
      isLoading: false,
      error: null,
    };
  });

  useEffect(() => {
    saveSession(session);
  }, [session]);

  const sendMessage = async (message: string) => {
    const userMessage: ChatMessage = {
      id: crypto.randomUUID(),
      role: 'user',
      content: message,
      timestamp: new Date().toISOString(),
    };

    setSession((prev) => ({
      ...prev,
      messages: [...prev.messages, userMessage],
      isLoading: true,
      error: null,
    }));

    try {
      if (config.streamingEnabled) {
        let streamingMessage: ChatMessage = {
          id: crypto.randomUUID(),
          role: 'assistant',
          content: '',
          timestamp: new Date().toISOString(),
          isStreaming: true,
        };

        setSession((prev) => ({
          ...prev,
          messages: [...prev.messages, streamingMessage],
        }));

        await streamChatMessage(
          {
            message,
            session_id: session.sessionId,
            context: session.selectedContext || undefined,
          },
          config,
          {
            onToken: (token) => {
              streamingMessage.content += token;
              setSession((prev) => ({
                ...prev,
                messages: prev.messages.map((m) =>
                  m.id === streamingMessage.id ? { ...streamingMessage } : m
                ),
              }));
            },
            onDone: (response) => {
              streamingMessage = {
                ...streamingMessage,
                content: response.response,
                confidence: response.confidence,
                confidenceLevel: response.confidence_level,
                sources: response.sources,
                isStreaming: false,
              };
              setSession((prev) => ({
                ...prev,
                messages: prev.messages.map((m) =>
                  m.id === streamingMessage.id ? streamingMessage : m
                ),
                isLoading: false,
              }));
            },
            onError: (error) => {
              setSession((prev) => ({
                ...prev,
                error: error.message,
                isLoading: false,
              }));
            },
          }
        );
      } else {
        const response = await sendChatMessage(
          {
            message,
            session_id: session.sessionId,
            context: session.selectedContext || undefined,
          },
          config
        );

        const assistantMessage: ChatMessage = {
          id: crypto.randomUUID(),
          role: 'assistant',
          content: response.response,
          timestamp: new Date().toISOString(),
          confidence: response.confidence,
          confidenceLevel: response.confidence_level,
          sources: response.sources,
        };

        setSession((prev) => ({
          ...prev,
          messages: [...prev.messages, assistantMessage],
          isLoading: false,
        }));
      }
    } catch (error) {
      setSession((prev) => ({
        ...prev,
        error: error.message,
        isLoading: false,
      }));
    }
  };

  const clearHistory = () => {
    setSession({
      sessionId: generateSessionId(),
      messages: [],
      createdAt: new Date().toISOString(),
      lastUpdatedAt: new Date().toISOString(),
      selectedContext: null,
      isLoading: false,
      error: null,
    });
  };

  const setSelectedContext = (text: string | null) => {
    setSession((prev) => ({ ...prev, selectedContext: text }));
  };

  return (
    <ChatContext.Provider value={{ session, sendMessage, clearHistory, setSelectedContext, config }}>
      {children}
    </ChatContext.Provider>
  );
}

export function useChatContext() {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChatContext must be used within ChatProvider');
  }
  return context;
}
```

---

## Step 9: Integrate into Docusaurus

Edit `src/theme/Root.tsx` (create if doesn't exist):

```typescript
// src/theme/Root.tsx
import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import RAGChatbot from '@site/src/components/RAGChatbot';

export default function Root({ children }) {
  const { siteConfig } = useDocusaurusContext();

  const backendConfig = {
    baseUrl: siteConfig.customFields.ragBackendUrl as string,
    apiKey: siteConfig.customFields.ragApiKey as string | undefined,
    timeoutMs: 30000,
    retryAttempts: 2,
    streamingEnabled: true,
  };

  return (
    <>
      {children}
      <RAGChatbot config={backendConfig} />
    </>
  );
}
```

---

## Step 10: Test the Integration

Start the Docusaurus development server:

```bash
npm run start
```

**Manual Test Cases** (from spec.md):

1. **Basic Q&A** (US1):
   - Open any page
   - Click chat icon (bottom-right)
   - Type: "What is a ROS 2 node?"
   - Press Enter
   - Verify: Response appears with citations within 5 seconds

2. **Text Selection** (US2):
   - Select a paragraph on the page
   - Press Cmd+K (Mac) or Ctrl+K (Windows)
   - Verify: Chatbot opens with selected text shown
   - Type: "Explain this in simpler terms"
   - Verify: Response references the selected content

3. **Conversation History** (US3):
   - Ask: "What is URDF?"
   - Receive answer
   - Ask: "Can you show an example?"
   - Verify: Response understands "example" refers to URDF

4. **Streaming Response** (US4):
   - Ask a question that generates a long response
   - Verify: Words appear incrementally (not all at once)
   - Verify: First token appears within 1 second

5. **Error Handling** (from FR-005):
   - Stop backend (Ctrl+C on backend terminal)
   - Ask a question
   - Verify: User-friendly error message appears
   - Restart backend
   - Click "Retry"
   - Verify: Response arrives successfully

---

## Step 11: Verify Success Criteria

Check that all measurable outcomes are met:

- ✅ **SC-001**: Response latency < 5 seconds (check browser Network tab)
- ✅ **SC-002**: Interface loads < 1 second (check Lighthouse)
- ✅ **SC-003**: Errors display user-friendly messages (test by stopping backend)
- ✅ **SC-004**: Text selection works up to 5000 characters (select large block)
- ✅ **SC-005**: History persists across page navigations (navigate to different chapter)
- ✅ **SC-006**: Streaming first token < 1 second (check Network timing)
- ✅ **SC-007**: Keyboard accessible (Tab to chat icon, Enter to open, Esc to close)
- ✅ **SC-008**: Mobile responsive (resize browser to 320px width)
- ✅ **SC-009**: Citations are clickable (click source link)
- ✅ **SC-010**: Setup time < 30 minutes (measure from Step 1 to working chatbot)

---

## Common Issues & Troubleshooting

### Issue 1: CORS Error

**Symptom**: Console error: "CORS policy: No 'Access-Control-Allow-Origin' header"

**Solution**:
```python
# In backend main.py, verify CORS middleware:
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Add your frontend origin
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Content-Type", "X-API-Key"],
)
```

### Issue 2: Backend Not Responding

**Symptom**: Timeout errors after 30 seconds

**Solution**:
```bash
# Check backend is running
curl http://localhost:8000/health

# Check backend logs
tail -f backend/logs/app.log

# Restart backend with verbose logging
cd backend
uvicorn agent_api.main:app --reload --log-level debug
```

### Issue 3: Streaming Not Working

**Symptom**: Response appears all at once, not incrementally

**Solution**:
- Check browser compatibility (Chrome/Firefox/Safari/Edge only)
- Verify `@microsoft/fetch-event-source` is installed
- Check Network tab → Response type should be "eventsource"
- If streaming fails, frontend falls back to sync automatically

### Issue 4: Session Not Persisting

**Symptom**: Conversation history lost on page navigation

**Solution**:
- Check browser console for sessionStorage errors
- Verify `STORAGE_KEY` matches in ChatContext.tsx
- Check browser privacy settings (sessionStorage must be enabled)

### Issue 5: Citations Not Appearing

**Symptom**: Response displays but no source citations

**Solution**:
- Check backend is returning `sources` array in response
- Verify `ChatMessage` component renders `sources` prop
- Check console for rendering errors in `Source` component

---

## Next Steps

Once basic integration is working:

1. **Styling**: Customize `styles.module.css` to match site theme
2. **Accessibility**: Test with screen reader (NVDA, JAWS, VoiceOver)
3. **Mobile**: Test on real devices (iOS Safari, Android Chrome)
4. **Performance**: Run Lighthouse audit (target: 90+ score)
5. **Production**: Deploy with HTTPS backend URL in environment variables

---

## Additional Resources

- **Full Specification**: `specs/004-rag-chatbot-frontend/spec.md`
- **Data Models**: `specs/004-rag-chatbot-frontend/data-model.md`
- **API Contract**: `specs/004-rag-chatbot-frontend/contracts/backend-api.md`
- **Backend Docs**: `backend/README.md`
- **Docusaurus Docs**: https://docusaurus.io/docs

---

## Support

If you encounter issues:
1. Check backend logs: `backend/logs/app.log`
2. Check frontend console: Browser DevTools → Console
3. Verify environment variables: `echo $RAG_BACKEND_URL`
4. Test backend directly: `curl http://localhost:8000/chat/run -d '...'`
5. Review error messages and match to common issues above

**Estimated Completion Time**: 20-30 minutes for experienced React/TypeScript developers.
