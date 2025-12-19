/**
 * Chat Provider component for RAG Chatbot
 * Feature: 004-rag-chatbot-frontend
 */

import React, { useState, useEffect, useCallback, ReactNode, useRef } from 'react';
import { ChatContext } from './ChatContext';
import type { ChatSession, ChatMessage, BackendConfig } from './types';
import { STORAGE_KEYS } from './types';
import { generateSessionId } from './utils';
import { sendChatMessageWithRetry, streamChatMessage } from './api';

interface ChatProviderProps {
  children: ReactNode;
  config: BackendConfig;
}

/**
 * Load session from sessionStorage (browser only)
 */
function loadSession(): ChatSession | null {
  if (typeof window === 'undefined') return null;

  try {
    const stored = sessionStorage.getItem(STORAGE_KEYS.CHAT_SESSION);
    return stored ? JSON.parse(stored) : null;
  } catch (error) {
    console.error('[RAGChatbot] Failed to load session:', error);
    return null;
  }
}

/**
 * Save session to sessionStorage (browser only)
 */
function saveSession(session: ChatSession): void {
  if (typeof window === 'undefined') return;

  try {
    sessionStorage.setItem(STORAGE_KEYS.CHAT_SESSION, JSON.stringify(session));
  } catch (error) {
    console.error('[RAGChatbot] Failed to save session:', error);
  }
}

/**
 * Load panel state from sessionStorage (browser only)
 */
function loadPanelState(): boolean {
  if (typeof window === 'undefined') return false;

  try {
    const stored = sessionStorage.getItem(STORAGE_KEYS.PANEL_STATE);
    return stored === 'true';
  } catch (error) {
    return false;
  }
}

/**
 * Save panel state to sessionStorage (browser only)
 */
function savePanelState(isOpen: boolean): void {
  if (typeof window === 'undefined') return;

  try {
    sessionStorage.setItem(STORAGE_KEYS.PANEL_STATE, isOpen.toString());
  } catch (error) {
    console.error('[RAGChatbot] Failed to save panel state:', error);
  }
}

/**
 * ChatProvider component - manages chat session state and provides context
 */
export function ChatProvider({ children, config }: ChatProviderProps): JSX.Element {
  // Initialize session from sessionStorage or create new
  const [session, setSession] = useState<ChatSession>(() => {
    const loaded = loadSession();
    if (loaded) {
      return loaded;
    }

    const now = new Date().toISOString();
    return {
      sessionId: generateSessionId(),
      messages: [],
      createdAt: now,
      lastUpdatedAt: now,
      selectedContext: null,
      isLoading: false,
      error: null,
    };
  });

  // Panel open/closed state
  const [isPanelOpen, setIsPanelOpen] = useState<boolean>(() => loadPanelState());

  // Save session to sessionStorage whenever it changes
  useEffect(() => {
    saveSession(session);
  }, [session]);

  // Save panel state whenever it changes
  useEffect(() => {
    savePanelState(isPanelOpen);
  }, [isPanelOpen]);

  // Ref to track current streaming assistant message ID
  const streamingMessageIdRef = useRef<string | null>(null);

  /**
   * Send a message to the backend (with streaming support)
   */
  const sendMessage = useCallback(
    async (message: string): Promise<void> => {
      // Create user message
      const userMessage: ChatMessage = {
        id: generateSessionId(),
        role: 'user',
        content: message,
        timestamp: new Date().toISOString(),
      };

      // Add user message and set loading state
      setSession((prev) => ({
        ...prev,
        messages: [...prev.messages, userMessage],
        isLoading: true,
        error: null,
        lastUpdatedAt: new Date().toISOString(),
      }));

      // Prepare request with history (last 5 messages)
      const history = session.messages
        .slice(-5)
        .map((msg) => ({ role: msg.role, content: msg.content }));

      const request = {
        message,
        session_id: session.sessionId,
        context: session.selectedContext || undefined,
        history: history.length > 0 ? history : undefined,
        stream: config.streamingEnabled, // Set stream flag based on config
      };

      // Use streaming if enabled
      if (config.streamingEnabled) {
        // Create empty assistant message for streaming
        const assistantMessageId = generateSessionId();
        streamingMessageIdRef.current = assistantMessageId;

        const streamingMessage: ChatMessage = {
          id: assistantMessageId,
          role: 'assistant',
          content: '',
          timestamp: new Date().toISOString(),
          isStreaming: true,
        };

        // Add streaming message placeholder
        setSession((prev) => ({
          ...prev,
          messages: [...prev.messages, streamingMessage],
          isLoading: false, // Not loading, but streaming
        }));

        // Stream the response
        await streamChatMessage(request, config, {
          onToken: (token: string) => {
            setSession((prev) => {
              const messages = [...prev.messages];
              const lastMessage = messages[messages.length - 1];

              if (lastMessage && lastMessage.id === assistantMessageId) {
                lastMessage.content += token;
              }

              return {
                ...prev,
                messages,
                lastUpdatedAt: new Date().toISOString(),
              };
            });
          },

          onDone: (response) => {
            setSession((prev) => {
              const messages = [...prev.messages];
              const lastMessage = messages[messages.length - 1];

              if (lastMessage && lastMessage.id === assistantMessageId) {
                // Update with final metadata
                lastMessage.content = response.response;
                lastMessage.confidence = response.confidence;
                lastMessage.confidenceLevel = response.confidence_level;
                lastMessage.sources = response.sources;
                lastMessage.isStreaming = false;
              }

              streamingMessageIdRef.current = null;

              return {
                ...prev,
                messages,
                isLoading: false,
                error: null,
                selectedContext: null, // Clear context after successful message
                lastUpdatedAt: new Date().toISOString(),
              };
            });
          },

          onError: (error) => {
            streamingMessageIdRef.current = null;

            setSession((prev) => {
              // Remove the streaming message on error
              const messages = prev.messages.filter((msg) => msg.id !== assistantMessageId);

              return {
                ...prev,
                messages,
                isLoading: false,
                error: error.message || 'An error occurred. Please try again.',
                lastUpdatedAt: new Date().toISOString(),
              };
            });
          },
        });
      } else {
        // Use synchronous API
        try {
          const response = await sendChatMessageWithRetry(request, config);

          // Create assistant message
          const assistantMessage: ChatMessage = {
            id: generateSessionId(),
            role: 'assistant',
            content: response.response,
            timestamp: new Date().toISOString(),
            confidence: response.confidence,
            confidenceLevel: response.confidence_level,
            sources: response.sources,
          };

          // Add assistant message and clear loading/error
          setSession((prev) => ({
            ...prev,
            messages: [...prev.messages, assistantMessage],
            isLoading: false,
            error: null,
            selectedContext: null, // Clear context after successful message
            lastUpdatedAt: new Date().toISOString(),
          }));
        } catch (error: any) {
          // Set error state
          setSession((prev) => ({
            ...prev,
            isLoading: false,
            error: error.message || 'An error occurred. Please try again.',
            lastUpdatedAt: new Date().toISOString(),
          }));
        }
      }
    },
    [session.sessionId, session.messages, session.selectedContext, config]
  );

  /**
   * Clear conversation history and start new session
   */
  const clearHistory = useCallback((): void => {
    const now = new Date().toISOString();
    setSession({
      sessionId: generateSessionId(),
      messages: [],
      createdAt: now,
      lastUpdatedAt: now,
      selectedContext: null,
      isLoading: false,
      error: null,
    });
  }, []);

  /**
   * Set selected text context
   */
  const setSelectedContext = useCallback((text: string | null): void => {
    setSession((prev) => ({
      ...prev,
      selectedContext: text,
      lastUpdatedAt: new Date().toISOString(),
    }));
  }, []);

  /**
   * Toggle chat panel open/closed
   */
  const togglePanel = useCallback((): void => {
    setIsPanelOpen((prev) => !prev);
  }, []);

  return (
    <ChatContext.Provider
      value={{
        session,
        sendMessage,
        clearHistory,
        setSelectedContext,
        config,
        togglePanel,
        isPanelOpen,
      }}
    >
      {children}
    </ChatContext.Provider>
  );
}
