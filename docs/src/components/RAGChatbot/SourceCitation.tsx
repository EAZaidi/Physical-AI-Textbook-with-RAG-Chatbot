/**
 * Source Citation component - displays textbook source references
 * Feature: 004-rag-chatbot-frontend
 * Task: T020
 */

import React, { useState } from 'react';
import type { Source } from './types';
import styles from './styles.module.css';

export interface SourceCitationProps {
  /** Source data from backend */
  source: Source;

  /** Index in the sources array */
  index: number;
}

/**
 * SourceCitation component - displays chapter, section, similarity score, and chunk excerpt
 */
export function SourceCitation({ source, index }: SourceCitationProps): JSX.Element {
  const [isExpanded, setIsExpanded] = useState(false);

  // Handle both camelCase and snake_case from backend
  const similarityScore = (source.similarityScore ?? (source as any).similarity_score ?? 0);

  // Format similarity score as percentage
  const similarityPercent = Math.round(similarityScore * 100);

  // Truncate chunk text for preview (handle undefined/null)
  const chunkText = source.chunkText || (source as any).chunk_text || '';
  const previewText = chunkText.length > 150
    ? `${chunkText.substring(0, 150)}...`
    : chunkText;

  return (
    <div className={styles.sourceCitation}>
      <div className={styles.sourceHeader}>
        <span className={styles.sourceNumber}>[{index + 1}]</span>
        <div className={styles.sourceInfo}>
          <span className={styles.sourceChapter}>{source.chapter}</span>
          {source.section && (
            <>
              <span className={styles.sourceSeparator}> › </span>
              <span className={styles.sourceSection}>{source.section}</span>
            </>
          )}
        </div>
        <span className={styles.sourceSimilarity} title={`Similarity score: ${similarityScore.toFixed(3)}`}>
          {similarityPercent}%
        </span>
      </div>

      <button
        type="button"
        className={styles.sourceExpandButton}
        onClick={() => setIsExpanded(!isExpanded)}
        aria-expanded={isExpanded}
        aria-label={isExpanded ? 'Collapse excerpt' : 'Expand excerpt'}
      >
        <span className={styles.sourcePreview}>
          {isExpanded ? chunkText : previewText}
        </span>
        <svg
          width="16"
          height="16"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
          className={isExpanded ? styles.sourceExpandIconOpen : styles.sourceExpandIconClosed}
          aria-hidden="true"
        >
          <polyline points="6 9 12 15 18 9" />
        </svg>
      </button>

      {source.url && (
        <a
          href={source.url}
          className={styles.sourceLink}
          target="_blank"
          rel="noopener noreferrer"
          aria-label={`View source: ${source.chapter}`}
        >
          View in textbook →
        </a>
      )}
    </div>
  );
}
