/**
 * Docusaurus Root theme wrapper - integrates RAG Chatbot globally
 * Feature: 004-rag-chatbot-frontend
 * Tasks: T025, T062 (lazy loading)
 */

import React, { ReactNode, Suspense, lazy } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { ChatProvider } from '../components/RAGChatbot/ChatProvider';
import { ChatbotErrorBoundary } from '../components/RAGChatbot/ErrorBoundary';
import { DEFAULT_BACKEND_CONFIG } from '../components/RAGChatbot/types';
import type { BackendConfig } from '../components/RAGChatbot/types';

// Lazy load the RAGChatbot component
const RAGChatbot = lazy(() => import('../components/RAGChatbot'));

interface RootProps {
  children: ReactNode;
}

/**
 * Root component - wraps entire Docusaurus site with ChatProvider
 */
export default function Root({ children }: RootProps): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  // Extract backend config from Docusaurus customFields
  const customFields = siteConfig.customFields || {};
  const backendConfig: BackendConfig = {
    ...DEFAULT_BACKEND_CONFIG,
    baseUrl: (customFields.ragBackendUrl as string) || DEFAULT_BACKEND_CONFIG.baseUrl,
    apiKey: (customFields.ragApiKey as string) || undefined,
  };

  // Validate backend URL in production
  if (
    typeof window !== 'undefined' &&
    process.env.NODE_ENV === 'production' &&
    backendConfig.baseUrl.startsWith('http://') &&
    !backendConfig.baseUrl.includes('localhost')
  ) {
    console.warn(
      '[RAGChatbot] WARNING: Using HTTP in production. Backend URL should use HTTPS:',
      backendConfig.baseUrl
    );
  }

  return (
    <>
      {children}
      <ChatbotErrorBoundary>
        <ChatProvider config={backendConfig}>
          <Suspense fallback={null}>
            <RAGChatbot />
          </Suspense>
        </ChatProvider>
      </ChatbotErrorBoundary>
    </>
  );
}
