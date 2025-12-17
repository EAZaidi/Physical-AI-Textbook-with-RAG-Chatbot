/**
 * Chat Panel component - main chat interface container
 * Feature: 004-rag-chatbot-frontend
 * Task: T015, T029
 */

import React, { useEffect, useRef } from 'react';
import { useChatContext } from './ChatContext';
import { ChatMessage } from './ChatMessage';
import { ChatInput } from './ChatInput';
import { ErrorMessage } from './ErrorMessage';
import { LoadingIndicator } from './LoadingIndicator';
import styles from './styles.module.css';

/**
 * ChatPanel component - contains header, messages, error, and input
 */
export function ChatPanel(): JSX.Element {
  const { session, clearHistory, togglePanel, sendMessage } = useChatContext();
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive or content updates (streaming)
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [session.messages, session.messages.length > 0 ? session.messages[session.messages.length - 1]?.content : '']);

  const handleRetry = () => {
    // Retry last user message
    const lastUserMessage = [...session.messages]
      .reverse()
      .find((msg) => msg.role === 'user');

    if (lastUserMessage) {
      sendMessage(lastUserMessage.content);
    }
  };

  const handleDismissError = () => {
    // Error dismissal is handled by clearing it from session in ChatProvider
    // For now, we can just trigger a state update by sending an empty update
    // In a more sophisticated setup, we'd have a dedicated clearError method
  };

  return (
    <div className={styles.chatPanel} role="dialog" aria-labelledby="chat-title">
      {/* Header */}
      <div className={styles.chatHeader}>
        <h2 id="chat-title" className={styles.chatTitle}>
          Ask AI about this textbook
        </h2>
        <div className={styles.chatHeaderActions}>
          <button
            type="button"
            className={styles.newConversationButton}
            onClick={clearHistory}
            aria-label="Start new conversation"
            title="New Conversation"
          >
            <svg
              width="18"
              height="18"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
              aria-hidden="true"
            >
              <path d="M12 5v14M5 12h14" />
            </svg>
            <span className={styles.buttonText}>New</span>
          </button>
          <button
            type="button"
            className={styles.closeButton}
            onClick={togglePanel}
            aria-label="Close chatbot"
          >
            <svg
              width="20"
              height="20"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
              aria-hidden="true"
            >
              <line x1="18" y1="6" x2="6" y2="18" />
              <line x1="6" y1="6" x2="18" y2="18" />
            </svg>
          </button>
        </div>
      </div>

      {/* Messages Container */}
      <div className={styles.messagesContainer}>
        <div className={styles.messagesList} role="log" aria-live="polite" aria-relevant="additions">
          {/* Welcome Message */}
          {session.messages.length === 0 && !session.isLoading && (
            <div className={styles.welcomeMessage}>
              <svg
                width="48"
                height="48"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="1.5"
                strokeLinecap="round"
                strokeLinejoin="round"
                className={styles.welcomeIcon}
                aria-hidden="true"
              >
                <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
              </svg>
              <h3 className={styles.welcomeTitle}>Welcome to AI Assistant</h3>
              <p className={styles.welcomeText}>
                Ask me anything about this textbook. I'll provide answers with citations to relevant sections.
              </p>
              <div className={styles.welcomeTips}>
                <p className={styles.welcomeTip}>
                  <strong>Tip:</strong> Select text on the page and press <kbd>Cmd/Ctrl+K</kbd> to ask about it.
                </p>
              </div>
            </div>
          )}

          {/* Messages */}
          {session.messages.map((message) => (
            <ChatMessage key={message.id} message={message} />
          ))}

          {/* Loading Indicator */}
          {session.isLoading && <LoadingIndicator />}

          {/* Scroll Anchor */}
          <div ref={messagesEndRef} />
        </div>

        {/* Error Display */}
        {session.error && (
          <ErrorMessage
            error={session.error}
            retryable={true}
            onRetry={handleRetry}
            onDismiss={handleDismissError}
          />
        )}
      </div>

      {/* Input Area */}
      <div className={styles.chatInputArea}>
        <ChatInput />
      </div>
    </div>
  );
}
