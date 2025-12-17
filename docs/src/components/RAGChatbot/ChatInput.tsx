/**
 * Chat Input component - textarea with submit button
 * Feature: 004-rag-chatbot-frontend
 * Task: T017
 */

import React, { useState, useRef, useEffect, KeyboardEvent } from 'react';
import { useChatContext } from './ChatContext';
import styles from './styles.module.css';

const MAX_MESSAGE_LENGTH = 1000;
const WARN_THRESHOLD = 800;

/**
 * ChatInput component - message input with auto-resize and character counter
 */
export function ChatInput(): JSX.Element {
  const { session, sendMessage, setSelectedContext } = useChatContext();
  const [inputValue, setInputValue] = useState('');
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  // Auto-resize textarea based on content
  useEffect(() => {
    const textarea = textareaRef.current;
    if (textarea) {
      textarea.style.height = 'auto';
      textarea.style.height = `${Math.min(textarea.scrollHeight, 120)}px`;
    }
  }, [inputValue]);

  // Auto-focus input when panel opens
  useEffect(() => {
    textareaRef.current?.focus();
  }, []);

  const handleSubmit = async (e?: React.FormEvent) => {
    e?.preventDefault();

    const trimmed = inputValue.trim();
    if (!trimmed || session.isLoading) return;

    if (trimmed.length > MAX_MESSAGE_LENGTH) {
      return;
    }

    setInputValue('');
    await sendMessage(trimmed);
  };

  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    // Submit on Enter (without Shift)
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  const handleDismissContext = () => {
    setSelectedContext(null);
  };

  const charCount = inputValue.length;
  const showCharCounter = charCount > WARN_THRESHOLD;
  const isOverLimit = charCount > MAX_MESSAGE_LENGTH;

  return (
    <form className={styles.chatInputForm} onSubmit={handleSubmit}>
      {/* Selected Context Chip */}
      {session.selectedContext && (
        <div className={styles.contextChip} role="status">
          <span className={styles.contextLabel}>Selected text:</span>
          <span className={styles.contextText} title={session.selectedContext}>
            {session.selectedContext.length > 100
              ? `${session.selectedContext.substring(0, 100)}...`
              : session.selectedContext}
          </span>
          <button
            type="button"
            className={styles.contextDismiss}
            onClick={handleDismissContext}
            aria-label="Clear selected context"
          >
            <svg
              width="14"
              height="14"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
            >
              <line x1="18" y1="6" x2="6" y2="18" />
              <line x1="6" y1="6" x2="18" y2="18" />
            </svg>
          </button>
        </div>
      )}

      {/* Input Container */}
      <div className={styles.chatInputContainer}>
        <textarea
          ref={textareaRef}
          className={styles.chatInput}
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="Ask a question about this textbook..."
          disabled={session.isLoading}
          aria-label="Type your question"
          rows={1}
        />

        {/* Character Counter */}
        {showCharCounter && (
          <span
            className={isOverLimit ? styles.charCounterError : styles.charCounter}
            aria-live="polite"
          >
            {charCount}/{MAX_MESSAGE_LENGTH}
          </span>
        )}

        {/* Submit Button */}
        <button
          type="submit"
          className={styles.chatSubmitButton}
          disabled={!inputValue.trim() || session.isLoading || isOverLimit}
          aria-label="Send message"
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
          >
            <line x1="22" y1="2" x2="11" y2="13" />
            <polygon points="22 2 15 22 11 13 2 9 22 2" />
          </svg>
        </button>
      </div>

      {/* Hint Text */}
      <div className={styles.chatInputHint}>
        Press Enter to send, Shift+Enter for new line
      </div>
    </form>
  );
}
