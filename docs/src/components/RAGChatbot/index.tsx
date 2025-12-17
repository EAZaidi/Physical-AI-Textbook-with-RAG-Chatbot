/**
 * Main RAGChatbot component - entry point for chatbot feature
 * Feature: 004-rag-chatbot-frontend
 * Tasks: T024, T031 (text selection for US2)
 */

import React, { useEffect } from 'react';
import { useChatContext } from './ChatContext';
import { ChatToggle } from './ChatToggle';
import { ChatPanel } from './ChatPanel';
import styles from './styles.module.css';

/**
 * RAGChatbot component - main chatbot interface
 * Handles text selection keyboard shortcut and renders toggle/panel
 */
export function RAGChatbot(): JSX.Element {
  const { isPanelOpen, togglePanel, setSelectedContext } = useChatContext();

  // Handle text selection keyboard shortcut (Cmd/Ctrl+K)
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      // Check for Cmd+K (Mac) or Ctrl+K (Windows/Linux)
      if ((e.metaKey || e.ctrlKey) && e.key === 'k') {
        e.preventDefault();

        const selection = window.getSelection();
        if (selection && selection.toString().length > 0) {
          const text = selection.toString().trim();

          // Validate max 5000 characters (SC-004)
          if (text.length <= 5000) {
            setSelectedContext(text);
            if (!isPanelOpen) {
              togglePanel();
            }
          } else {
            // Show error if text too long
            console.warn('[RAGChatbot] Selected text exceeds 5000 character limit');
            alert('Selected text is too long (max 5000 characters). Please select a shorter passage.');
          }
        } else {
          // No text selected, just open panel
          if (!isPanelOpen) {
            togglePanel();
          }
        }
      }

      // Close panel on Escape
      if (e.key === 'Escape' && isPanelOpen) {
        togglePanel();
      }
    };

    document.addEventListener('keydown', handleKeyDown);

    return () => {
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [isPanelOpen, togglePanel, setSelectedContext]);

  return (
    <div className={styles.ragChatbot} role="region" aria-label="AI chatbot">
      {/* Toggle Button (always visible) */}
      <ChatToggle />

      {/* Chat Panel (conditional) */}
      {isPanelOpen && <ChatPanel />}
    </div>
  );
}

// Default export for easier imports
export default RAGChatbot;
