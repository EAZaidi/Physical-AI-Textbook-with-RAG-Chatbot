/**
 * Error Boundary component for RAG Chatbot
 * Feature: 004-rag-chatbot-frontend
 * Task: T068
 */

import React, { Component, ReactNode, ErrorInfo } from 'react';

interface ErrorBoundaryProps {
  children: ReactNode;
}

interface ErrorBoundaryState {
  hasError: boolean;
  error: Error | null;
}

/**
 * Error Boundary - catches React errors and displays fallback UI
 */
export class ChatbotErrorBoundary extends Component<ErrorBoundaryProps, ErrorBoundaryState> {
  constructor(props: ErrorBoundaryProps) {
    super(props);
    this.state = {
      hasError: false,
      error: null,
    };
  }

  static getDerivedStateFromError(error: Error): ErrorBoundaryState {
    return {
      hasError: true,
      error,
    };
  }

  componentDidCatch(error: Error, errorInfo: ErrorInfo): void {
    // Log error to console in development
    if (process.env.NODE_ENV === 'development') {
      console.error('[RAGChatbot] Error boundary caught error:', error);
      console.error('[RAGChatbot] Error info:', errorInfo);
    }

    // In production, you could send this to an error tracking service
    // Example: Sentry.captureException(error, { extra: errorInfo });
  }

  handleReset = (): void => {
    this.setState({
      hasError: false,
      error: null,
    });

    // Reload the page to reset the chatbot state
    if (typeof window !== 'undefined') {
      window.location.reload();
    }
  };

  render(): ReactNode {
    if (this.state.hasError) {
      return (
        <div
          style={{
            position: 'fixed',
            bottom: '20px',
            right: '20px',
            backgroundColor: '#fff',
            border: '2px solid #d32f2f',
            borderRadius: '8px',
            padding: '16px',
            maxWidth: '300px',
            boxShadow: '0 4px 6px rgba(0, 0, 0, 0.1)',
            zIndex: 1000,
          }}
        >
          <h3 style={{ margin: '0 0 8px 0', color: '#d32f2f', fontSize: '16px' }}>
            Chatbot Error
          </h3>
          <p style={{ margin: '0 0 12px 0', fontSize: '14px', color: '#666' }}>
            The chatbot encountered an error. Please try refreshing the page.
          </p>
          {process.env.NODE_ENV === 'development' && this.state.error && (
            <details style={{ fontSize: '12px', marginBottom: '12px' }}>
              <summary style={{ cursor: 'pointer', color: '#666' }}>Error details</summary>
              <pre
                style={{
                  fontSize: '11px',
                  overflow: 'auto',
                  maxHeight: '150px',
                  padding: '8px',
                  backgroundColor: '#f5f5f5',
                  borderRadius: '4px',
                  marginTop: '8px',
                }}
              >
                {this.state.error.message}
              </pre>
            </details>
          )}
          <button
            onClick={this.handleReset}
            style={{
              backgroundColor: '#d32f2f',
              color: '#fff',
              border: 'none',
              padding: '8px 16px',
              borderRadius: '4px',
              cursor: 'pointer',
              fontSize: '14px',
              width: '100%',
            }}
          >
            Reload Page
          </button>
        </div>
      );
    }

    return this.props.children;
  }
}
