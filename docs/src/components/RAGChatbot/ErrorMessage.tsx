/**
 * Error Message component - displays user-friendly error messages
 * Feature: 004-rag-chatbot-frontend
 * Task: T019
 */

import React from 'react';
import styles from './styles.module.css';

export interface ErrorMessageProps {
  /** Error message text */
  error: string;

  /** Whether error is retryable */
  retryable?: boolean;

  /** Callback when retry button is clicked */
  onRetry?: () => void;

  /** Callback when dismiss button is clicked */
  onDismiss: () => void;
}

/**
 * ErrorMessage component - displays errors with retry/dismiss options
 */
export function ErrorMessage({
  error,
  retryable = false,
  onRetry,
  onDismiss,
}: ErrorMessageProps): JSX.Element {
  return (
    <div className={styles.errorMessage} role="alert">
      <div className={styles.errorIcon} aria-hidden="true">
        <svg
          width="20"
          height="20"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        >
          <circle cx="12" cy="12" r="10" />
          <line x1="12" y1="8" x2="12" y2="12" />
          <line x1="12" y1="16" x2="12.01" y2="16" />
        </svg>
      </div>
      <div className={styles.errorContent}>
        <p className={styles.errorText}>{error}</p>
        <div className={styles.errorActions}>
          {retryable && onRetry && (
            <button
              type="button"
              className={styles.errorRetryButton}
              onClick={onRetry}
              aria-label="Retry request"
            >
              Retry
            </button>
          )}
          <button
            type="button"
            className={styles.errorDismissButton}
            onClick={onDismiss}
            aria-label="Dismiss error"
          >
            Dismiss
          </button>
        </div>
      </div>
    </div>
  );
}
