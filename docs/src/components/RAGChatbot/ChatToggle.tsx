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
          width="28"
          height="28"
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
        // Modern AI Chatbot Robot icon - white with green eye pupils
        <svg
          width="36"
          height="36"
          viewBox="0 0 32 32"
          fill="none"
          aria-hidden="true"
        >
          {/* Robot head - rounded square */}
          <rect x="8" y="10" width="16" height="16" rx="3" fill="white" stroke="rgba(255,255,255,0.5)" strokeWidth="1"/>

          {/* Antenna */}
          <circle cx="16" cy="6" r="2" fill="white"/>
          <rect x="15.5" y="6" width="1" height="4" fill="white"/>

          {/* Eyes - white with green pupils */}
          <circle cx="13" cy="16" r="2" fill="white"/>
          <circle cx="13" cy="16" r="1" fill="#2e8555"/>
          <circle cx="19" cy="16" r="2" fill="white"/>
          <circle cx="19" cy="16" r="1" fill="#2e8555"/>

          {/* Happy smile */}
          <path d="M 12 21 Q 16 23 20 21" stroke="#2e8555" strokeWidth="1.5" fill="none" strokeLinecap="round"/>

          {/* Ears/Side panels */}
          <rect x="6" y="14" width="2" height="6" rx="1" fill="white"/>
          <rect x="24" y="14" width="2" height="6" rx="1" fill="white"/>

          {/* Chat bubble indicator */}
          <circle cx="24" cy="12" r="3" fill="white" opacity="0.8"/>
          <circle cx="24" cy="12" r="1" fill="#2e8555"/>
        </svg>
      )}
    </button>
  );
}
