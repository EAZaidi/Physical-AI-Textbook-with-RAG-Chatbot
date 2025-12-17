/**
 * React Context for RAG Chatbot state management
 * Feature: 004-rag-chatbot-frontend
 */

import React, { createContext, useContext } from 'react';
import type { ChatSession, BackendConfig } from './types';

/**
 * Chat Context type definition
 */
export interface ChatContextType {
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

  /** Toggle chat panel open/closed */
  togglePanel: () => void;

  /** Check if chat panel is open */
  isPanelOpen: boolean;
}

/**
 * Chat Context - provides global state for chatbot functionality
 */
export const ChatContext = createContext<ChatContextType | undefined>(undefined);

/**
 * Hook to use Chat Context
 * @returns ChatContextType
 * @throws Error if used outside ChatProvider
 */
export function useChatContext(): ChatContextType {
  const context = useContext(ChatContext);

  if (!context) {
    throw new Error('useChatContext must be used within ChatProvider');
  }

  return context;
}
