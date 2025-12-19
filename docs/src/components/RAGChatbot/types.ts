/**
 * TypeScript interfaces for RAG Chatbot Frontend Integration
 * Feature: 004-rag-chatbot-frontend
 */

// ============================================================================
// Core Entities
// ============================================================================

/**
 * Represents a single message in the conversation thread
 */
export interface ChatMessage {
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

/**
 * Represents a conversation thread with multiple messages
 */
export interface ChatSession {
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

/**
 * Represents a citation/reference to textbook content
 */
export interface Source {
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

/**
 * Configuration for API communication with the FastAPI backend
 */
export interface BackendConfig {
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

/**
 * Default backend configuration values
 */
export const DEFAULT_BACKEND_CONFIG: BackendConfig = {
  baseUrl: 'https://essaabbas-rag-chatbot.hf.space',
  timeoutMs: 30000,
  retryAttempts: 2,
  streamingEnabled: false,
  headers: {
    'Content-Type': 'application/json',
  },
};

// ============================================================================
// Supporting Types (API Request/Response)
// ============================================================================

/**
 * Request payload for backend API calls
 */
export interface ChatRequest {
  /** User's question or message */
  message: string;

  /** Session identifier for conversation continuity */
  session_id: string;

  /** Optional selected text context */
  context?: string;

  /** Optional conversation history for multi-turn context */
  history?: Array<{ role: 'user' | 'assistant'; content: string }>;

  /** Enable streaming response (required for /chat/stream endpoint) */
  stream?: boolean;
}

/**
 * Response payload from backend API (synchronous endpoint)
 */
export interface ChatResponse {
  /** Generated response text (markdown formatted) */
  response: string;

  /** Overall confidence score (0.0 to 1.0) */
  confidence: number;

  /** Confidence level categorization */
  confidence_level: 'high' | 'medium' | 'low' | 'insufficient';

  /** Source citations */
  sources: Source[];

  /** Session identifier (echoed back) */
  session_id: string;

  /** Whether the agent decided to answer */
  should_answer: boolean;

  /** Optional refusal reason if shouldAnswer is false */
  refusal_reason?: string;
}

// ============================================================================
// Streaming Types (SSE)
// ============================================================================

/**
 * SSE event types for streaming responses
 */
export type StreamEventType = 'tool_call' | 'retrieval' | 'content' | 'done' | 'error';

/**
 * SSE event structure
 */
export interface StreamEvent {
  /** Event type */
  event: StreamEventType;

  /** Event payload (varies by event type) */
  data: StreamEventData;
}

/**
 * Union type for all possible stream event data
 */
export type StreamEventData =
  | ToolCallData
  | RetrievalData
  | ContentData
  | DoneData
  | ErrorData;

/**
 * Tool call event data (agent is calling a tool)
 */
export interface ToolCallData {
  toolName: string;
  arguments: Record<string, any>;
}

/**
 * Retrieval event data (agent retrieved relevant chunks)
 */
export interface RetrievalData {
  query: string;
  results: Source[];
}

/**
 * Content event data (incremental response token)
 */
export interface ContentData {
  delta: string; // Incremental text token
}

/**
 * Done event data (stream complete with final metadata)
 */
export interface DoneData {
  response: string; // Complete response
  confidence: number;
  confidenceLevel: string;
  sources: Source[];
  sessionId: string;
}

/**
 * Error event data (stream error)
 */
export interface ErrorData {
  error: string;
  code?: string;
}

// ============================================================================
// Error Types
// ============================================================================

/**
 * Classification of errors that can occur
 */
export interface ChatError {
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

// ============================================================================
// Storage Constants
// ============================================================================

/**
 * SessionStorage keys for persistence
 */
export const STORAGE_KEYS = {
  /** Current chat session */
  CHAT_SESSION: 'rag-chatbot-session',

  /** Chat panel open/closed state */
  PANEL_STATE: 'rag-chatbot-panel-state',

  /** User preferences (optional) */
  PREFERENCES: 'rag-chatbot-preferences',
} as const;
