/**
 * Loading Indicator component - shows loading state with animation
 * Feature: 004-rag-chatbot-frontend
 * Task: T018
 */

import React from 'react';
import styles from './styles.module.css';

export interface LoadingIndicatorProps {
  /** Optional loading message text */
  message?: string;
}

/**
 * LoadingIndicator component - animated spinner with text
 */
export function LoadingIndicator({ message = 'Thinking...' }: LoadingIndicatorProps): JSX.Element {
  return (
    <div className={styles.loadingIndicator} role="status" aria-live="polite">
      <div className={styles.spinner} aria-hidden="true">
        <svg
          width="24"
          height="24"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        >
          <circle cx="12" cy="12" r="10" opacity="0.25" />
          <path d="M12 2a10 10 0 0 1 10 10" opacity="0.75">
            <animateTransform
              attributeName="transform"
              type="rotate"
              from="0 12 12"
              to="360 12 12"
              dur="1s"
              repeatCount="indefinite"
            />
          </path>
        </svg>
      </div>
      <span className={styles.loadingText}>{message}</span>
    </div>
  );
}
