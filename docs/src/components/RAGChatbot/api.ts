/**
 * API client for RAG Chatbot Frontend Integration
 * Feature: 004-rag-chatbot-frontend
 * Tasks: T045, T046 (streaming support)
 */

import { fetchEventSource } from '@microsoft/fetch-event-source';
import type {
  ChatRequest,
  ChatResponse,
  BackendConfig,
  ChatError,
  StreamEventType
} from './types';
import { validateMessage, validateSessionId, validateContext } from './utils';

/**
 * Streaming callbacks for SSE events
 */
export interface StreamingCallbacks {
  /** Called for each content delta (token) */
  onToken: (token: string) => void;

  /** Called when streaming completes successfully */
  onDone: (response: ChatResponse) => void;

  /** Called when an error occurs */
  onError: (error: Error) => void;

  /** Called for tool_call events (optional, for debugging) */
  onToolCall?: (toolName: string, args: any) => void;

  /** Called for retrieval events (optional, for debugging) */
  onRetrieval?: (query: string, results: any[]) => void;
}

/**
 * Send a chat message to the backend API (synchronous)
 * @param request - Chat request payload
 * @param config - Backend configuration
 * @returns Promise resolving to chat response
 * @throws ChatError if request fails
 */
export async function sendChatMessage(
  request: ChatRequest,
  config: BackendConfig
): Promise<ChatResponse> {
  // Validate request (T063)
  const messageValidation = validateMessage(request.message);
  if (!messageValidation.isValid) {
    throw createChatError(400, messageValidation.error || 'Invalid message', false);
  }

  if (!validateSessionId(request.session_id)) {
    throw createChatError(400, 'Invalid session ID format', false);
  }

  if (request.context) {
    const contextValidation = validateContext(request.context);
    if (!contextValidation.isValid) {
      throw createChatError(400, contextValidation.error || 'Invalid context', false);
    }
  }

  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), config.timeoutMs);

  try {
    const response = await fetch(`${config.baseUrl}/chat/run`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        ...(config.apiKey ? { 'X-API-Key': config.apiKey } : {}),
        ...(config.headers || {}),
      },
      body: JSON.stringify(request),
      signal: controller.signal,
    });

    clearTimeout(timeoutId);

    if (!response.ok) {
      const error = await response.json().catch(() => ({ detail: 'Unknown error' }));

      // Extract error message from response
      // Backend may return: { detail: "string" } or { detail: { message: "string", error_code: "...", ... } }
      let errorMessage: string;
      if (typeof error.detail === 'string') {
        errorMessage = error.detail;
      } else if (error.detail && typeof error.detail === 'object' && 'message' in error.detail) {
        errorMessage = error.detail.message;
      } else {
        errorMessage = `HTTP ${response.status}`;
      }

      // Handle rate limiting (T064)
      if (response.status === 429) {
        const retryAfter = response.headers.get('Retry-After');
        const retrySeconds = retryAfter ? parseInt(retryAfter, 10) : 60;
        throw createChatError(
          429,
          `Too many questions. Please wait ${retrySeconds} seconds before trying again.`,
          true
        );
      }

      throw createChatError(
        response.status,
        errorMessage,
        true
      );
    }

    const data = await response.json();
    return transformBackendResponse(data);
  } catch (error) {
    clearTimeout(timeoutId);

    if (error instanceof Error) {
      if (error.name === 'AbortError') {
        throw createChatError(504, 'Request timeout', true);
      }

      // Network error (CORS, DNS, connection refused)
      if (error.message.includes('Failed to fetch') || error.message.includes('NetworkError')) {
        throw createChatError(
          0,
          'Unable to connect to the chatbot service. Please check your connection.',
          true
        );
      }
    }

    throw error;
  }
}

/**
 * Send a chat message with retry logic
 * @param request - Chat request payload
 * @param config - Backend configuration
 * @param attempt - Current attempt number (for internal recursion)
 * @returns Promise resolving to chat response
 */
export async function sendChatMessageWithRetry(
  request: ChatRequest,
  config: BackendConfig,
  attempt: number = 0
): Promise<ChatResponse> {
  try {
    return await sendChatMessage(request, config);
  } catch (error) {
    const chatError = error as ChatError;

    // Only retry if error is retryable and we haven't exceeded max attempts
    if (chatError.retryable && attempt < config.retryAttempts) {
      // Exponential backoff: 2^attempt * 1000ms (1s, 2s, 4s)
      const delay = Math.pow(2, attempt) * 1000;
      await new Promise((resolve) => setTimeout(resolve, delay));

      return sendChatMessageWithRetry(request, config, attempt + 1);
    }

    throw error;
  }
}

/**
 * Transform backend response from snake_case to camelCase
 * @param response - Backend response with snake_case fields
 * @returns Transformed response with camelCase fields
 */
function transformBackendResponse(response: any): ChatResponse {
  return {
    response: response.response || '',
    confidence: response.confidence || 0,
    confidence_level: response.confidence_level || 'insufficient',
    sources: (response.sources || []).map((source: any) => ({
      chunkText: source.chunk_text || source.chunkText || '',
      similarityScore: source.similarity_score ?? source.similarityScore ?? 0,
      chapter: source.chapter || 'Unknown',
      section: source.section || 'Unknown',
      url: source.url,
      chunkIndex: source.chunk_index ?? source.chunkIndex ?? 0,
      pageNumber: source.page_number ?? source.pageNumber,
    })),
    session_id: response.session_id || '',
    should_answer: response.should_answer !== false,
    refusal_reason: response.refusal_reason,
    timestamp: response.timestamp,
  };
}

/**
 * Create a standardized ChatError object that extends Error
 * @param statusCode - HTTP status code (0 for network errors)
 * @param message - Error message
 * @param retryable - Whether the error is retryable
 * @returns Error object with ChatError properties
 */
function createChatError(statusCode: number, message: string, retryable: boolean): Error & ChatError {
  let type: ChatError['type'] = 'unknown';

  if (statusCode === 0) {
    type = 'network';
  } else if (statusCode === 400) {
    type = 'validation';
  } else if (statusCode === 429 || statusCode === 503) {
    type = 'backend';
  } else if (statusCode === 504) {
    type = 'timeout';
  }

  const error = new Error(message) as Error & ChatError;
  error.type = type;
  error.statusCode = statusCode > 0 ? statusCode : undefined;
  error.retryable = retryable;

  return error;
}

/**
 * Stream a chat message from the backend API using Server-Sent Events
 * @param request - Chat request payload
 * @param config - Backend configuration
 * @param callbacks - Streaming callbacks
 * @returns Promise that resolves when streaming completes
 */
export async function streamChatMessage(
  request: ChatRequest,
  config: BackendConfig,
  callbacks: StreamingCallbacks
): Promise<void> {
  // Fallback to synchronous if streaming is disabled
  if (!config.streamingEnabled) {
    try {
      const response = await sendChatMessage(request, config);
      callbacks.onDone(response);
    } catch (error) {
      callbacks.onError(error as Error);
    }
    return;
  }

  const controller = new AbortController();
  let hasError = false;

  try {
    await fetchEventSource(`${config.baseUrl}/chat/stream`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        ...(config.apiKey ? { 'X-API-Key': config.apiKey } : {}),
        ...(config.headers || {}),
      },
      body: JSON.stringify(request),
      signal: controller.signal,

      onopen: async (response) => {
        if (!response.ok) {
          const error = await response.json().catch(() => ({ detail: 'Unknown error' }));
          throw new Error(error.detail || `HTTP ${response.status}`);
        }
      },

      onmessage: (event) => {
        try {
          // Parse SSE event
          const data = JSON.parse(event.data);

          // Handle different event types
          switch (event.event as StreamEventType) {
            case 'tool_call':
              if (callbacks.onToolCall) {
                callbacks.onToolCall(data.tool_name, data.arguments);
              }
              break;

            case 'retrieval':
              if (callbacks.onRetrieval) {
                callbacks.onRetrieval(data.query, data.results);
              }
              break;

            case 'content':
              if (data.delta) {
                callbacks.onToken(data.delta);
              }
              break;

            case 'done':
              // Final response with metadata - transform from snake_case
              const transformedResponse = transformBackendResponse({
                response: data.response || '',
                confidence: data.confidence || 0,
                confidence_level: data.confidence_level || 'insufficient',
                sources: data.sources || [],
                session_id: data.session_id || request.session_id,
                should_answer: data.should_answer !== false,
                refusal_reason: data.refusal_reason,
              });
              callbacks.onDone(transformedResponse);
              controller.abort(); // Close the connection
              break;

            case 'error':
              hasError = true;
              callbacks.onError(new Error(data.error || 'Streaming error'));
              controller.abort();
              break;

            default:
              console.warn('[RAGChatbot] Unknown SSE event type:', event.event);
          }
        } catch (error) {
          console.error('[RAGChatbot] Failed to parse SSE event:', error);
          callbacks.onError(error as Error);
        }
      },

      onerror: (error) => {
        if (!hasError) {
          console.warn('[RAGChatbot] SSE connection error, falling back to synchronous:', error);

          // Fallback to synchronous request (already transforms response)
          sendChatMessage(request, config)
            .then((response) => callbacks.onDone(response))
            .catch((error) => callbacks.onError(error));
        }

        // Throw error to stop fetchEventSource from retrying
        throw error;
      },

      openWhenHidden: true, // Continue streaming even when tab is hidden
    });
  } catch (error) {
    if (!hasError && (error as Error).name !== 'AbortError') {
      callbacks.onError(error as Error);
    }
  }
}

/**
 * Test backend health
 * @param config - Backend configuration
 * @returns Promise resolving to true if healthy, false otherwise
 */
export async function testBackendHealth(config: BackendConfig): Promise<boolean> {
  try {
    const response = await fetch(`${config.baseUrl}/health`, {
      method: 'GET',
      signal: AbortSignal.timeout(5000), // 5 second timeout
    });

    if (!response.ok) {
      return false;
    }

    const data = await response.json();
    return data.status === 'healthy';
  } catch (error) {
    return false;
  }
}
