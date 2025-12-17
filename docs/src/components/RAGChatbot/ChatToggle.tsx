/**
 * Chat Toggle Button component - floating button to open/close chatbot
 * Feature: 004-rag-chatbot-frontend
 * Task: T014
 */

import React from 'react';
import { useChatContext } from './ChatContext';
import styles from './styles.module.css';

/**
 * ChatToggle component - floating button in bottom-right corner
 */
export function ChatToggle(): JSX.Element {
  const { togglePanel, isPanelOpen } = useChatContext();

  return (
    <button
      className={styles.chatToggle}
      onClick={togglePanel}
      aria-label={isPanelOpen ? 'Close chatbot' : 'Open chatbot'}
      aria-expanded={isPanelOpen}
      type="button"
    >
      {isPanelOpen ? (
        // Close icon (X)
        <svg
          width="24"
          height="24"
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
      ) : (
        // Chat icon (message bubble)
        <svg
          width="24"
          height="24"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
          aria-hidden="true"
        >
          <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        </svg>
      )}
      <span className={styles.chatToggleText}>
        {isPanelOpen ? 'Close' : 'Ask AI'}
      </span>
    </button>
  );
}
