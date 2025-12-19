/**
 * Chat Message component - displays individual messages
 * Feature: 004-rag-chatbot-frontend
 * Tasks: T016, T021 (with react-markdown integration)
 */

import React from 'react';
import ReactMarkdown from 'react-markdown';
import remarkGfm from 'remark-gfm';
import type { ChatMessage as ChatMessageType } from './types';
import { SourceCitation } from './SourceCitation';
import styles from './styles.module.css';

export interface ChatMessageProps {
  /** Message data */
  message: ChatMessageType;
}

/**
 * Format timestamp as relative time (e.g., "2 minutes ago")
 */
function formatTimestamp(isoString: string): string {
  const date = new Date(isoString);
  const now = new Date();
  const diffMs = now.getTime() - date.getTime();
  const diffSec = Math.floor(diffMs / 1000);
  const diffMin = Math.floor(diffSec / 60);
  const diffHour = Math.floor(diffMin / 60);

  if (diffSec < 60) return 'Just now';
  if (diffMin < 60) return `${diffMin} minute${diffMin !== 1 ? 's' : ''} ago`;
  if (diffHour < 24) return `${diffHour} hour${diffHour !== 1 ? 's' : ''} ago`;

  return date.toLocaleDateString();
}

/**
 * Confidence badge component
 */
function ConfidenceBadge({ level }: { level: string }): JSX.Element {
  const labels: Record<string, { text: string; className: string }> = {
    high: { text: 'High confidence', className: styles.confidenceHigh },
    medium: { text: 'Medium confidence', className: styles.confidenceMedium },
    low: { text: 'Low confidence', className: styles.confidenceLow },
    insufficient: { text: 'Low confidence', className: styles.confidenceInsufficient },
  };

  const badge = labels[level] || labels.medium;

  return (
    <span className={`${styles.confidenceBadge} ${badge.className}`} title={badge.text}>
      {badge.text}
    </span>
  );
}

/**
 * ChatMessage component - renders user or assistant messages
 * Memoized to prevent unnecessary re-renders
 */
export const ChatMessage = React.memo(function ChatMessage({ message }: ChatMessageProps): JSX.Element {
  const isUser = message.role === 'user';
  const messageClass = isUser ? styles.messageUser : styles.messageAssistant;

  return (
    <div className={`${styles.message} ${messageClass}`} role="article">
      {/* Message Content */}
      <div className={styles.messageContent}>
        {isUser ? (
          // User message: plain text
          <p className={styles.messageText}>{message.content}</p>
        ) : (
          // Assistant message: markdown
          <div className={styles.messageMarkdown}>
            <ReactMarkdown remarkPlugins={[remarkGfm]}>
              {message.content}
            </ReactMarkdown>
          </div>
        )}
      </div>

      {/* Message Metadata (Assistant only) */}
      {!isUser && (
        <div className={styles.messageMetadata}>
          {/* Confidence Badge - Hidden per user request */}
          {/* {message.confidenceLevel && (
            <ConfidenceBadge level={message.confidenceLevel} />
          )} */}

          {/* Timestamp */}
          <span className={styles.messageTimestamp} title={new Date(message.timestamp).toLocaleString()}>
            {formatTimestamp(message.timestamp)}
          </span>
        </div>
      )}

      {/* Sources (Assistant only) - Hidden per user request */}
      {/* {!isUser && message.sources && message.sources.length > 0 && (
        <div className={styles.messageSources}>
          <h4 className={styles.sourcesTitle}>Sources</h4>
          <div className={styles.sourcesList}>
            {message.sources.map((source, index) => (
              <SourceCitation key={index} source={source} index={index} />
            ))}
          </div>
        </div>
      )} */}

      {/* Streaming Indicator */}
      {message.isStreaming && (
        <div className={styles.streamingIndicator} aria-live="polite">
          <span className={styles.streamingDot} />
          <span className={styles.streamingDot} />
          <span className={styles.streamingDot} />
        </div>
      )}

      {/* Timestamp (User only) */}
      {isUser && (
        <span className={styles.messageTimestamp} title={new Date(message.timestamp).toLocaleString()}>
          {formatTimestamp(message.timestamp)}
        </span>
      )}
    </div>
  );
}, (prevProps, nextProps) => {
  // Custom comparison: only re-render if message content or metadata changed
  return (
    prevProps.message.id === nextProps.message.id &&
    prevProps.message.content === nextProps.message.content &&
    prevProps.message.isStreaming === nextProps.message.isStreaming &&
    prevProps.message.confidence === nextProps.message.confidence
  );
});
