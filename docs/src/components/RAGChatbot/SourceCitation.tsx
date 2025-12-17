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

  // Format similarity score as percentage
  const similarityPercent = Math.round(source.similarityScore * 100);

  // Truncate chunk text for preview
  const previewText = source.chunkText.length > 150
    ? `${source.chunkText.substring(0, 150)}...`
    : source.chunkText;

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
        <span className={styles.sourceSimilarity} title={`Similarity score: ${source.similarityScore.toFixed(3)}`}>
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
          {isExpanded ? source.chunkText : previewText}
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
