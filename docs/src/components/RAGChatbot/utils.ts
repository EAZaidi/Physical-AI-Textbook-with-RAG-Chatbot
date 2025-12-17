/**
 * Utility functions for RAG Chatbot Frontend Integration
 * Feature: 004-rag-chatbot-frontend
 */

/**
 * Generate a unique session ID using crypto.randomUUID()
 * @returns UUID v4 string
 */
export function generateSessionId(): string {
  return crypto.randomUUID();
}

/**
 * Validate a message string
 * @param message - Message to validate
 * @returns Object with isValid flag and optional error message
 */
export function validateMessage(message: string): { isValid: boolean; error?: string } {
  const trimmed = message.trim();

  if (trimmed.length === 0) {
    return { isValid: false, error: 'Please enter a question' };
  }

  if (trimmed.length > 1000) {
    return { isValid: false, error: 'Question is too long (max 1000 characters)' };
  }

  return { isValid: true };
}

/**
 * Validate a session ID (UUID v4 format)
 * @param sessionId - Session ID to validate
 * @returns True if valid UUID v4, false otherwise
 */
export function validateSessionId(sessionId: string): boolean {
  const uuidV4Regex = /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i;
  return uuidV4Regex.test(sessionId);
}

/**
 * Validate selected context text
 * @param context - Context text to validate
 * @returns Object with isValid flag and optional error message
 */
export function validateContext(context: string): { isValid: boolean; error?: string } {
  if (context.length > 5000) {
    return { isValid: false, error: 'Selected text is too long (max 5000 characters)' };
  }

  return { isValid: true };
}

/**
 * Format a timestamp to relative time (e.g., "2 minutes ago")
 * @param timestamp - ISO 8601 timestamp string
 * @returns Relative time string
 */
export function formatRelativeTime(timestamp: string): string {
  const now = new Date();
  const then = new Date(timestamp);
  const diffMs = now.getTime() - then.getTime();
  const diffSeconds = Math.floor(diffMs / 1000);
  const diffMinutes = Math.floor(diffSeconds / 60);
  const diffHours = Math.floor(diffMinutes / 60);
  const diffDays = Math.floor(diffHours / 24);

  if (diffSeconds < 60) {
    return 'just now';
  } else if (diffMinutes < 60) {
    return `${diffMinutes} minute${diffMinutes !== 1 ? 's' : ''} ago`;
  } else if (diffHours < 24) {
    return `${diffHours} hour${diffHours !== 1 ? 's' : ''} ago`;
  } else if (diffDays < 7) {
    return `${diffDays} day${diffDays !== 1 ? 's' : ''} ago`;
  } else {
    return then.toLocaleDateString();
  }
}

/**
 * Truncate text to a maximum length with ellipsis
 * @param text - Text to truncate
 * @param maxLength - Maximum length before truncation
 * @returns Truncated text with ellipsis if needed
 */
export function truncateText(text: string, maxLength: number): string {
  if (text.length <= maxLength) {
    return text;
  }
  return text.substring(0, maxLength) + '...';
}

/**
 * Map HTTP status code to user-friendly error message
 * @param statusCode - HTTP status code
 * @returns User-friendly error message
 */
export function getErrorMessage(statusCode: number): string {
  switch (statusCode) {
    case 400:
      return 'Your question is invalid. Please try rephrasing.';
    case 429:
      return 'Too many questions. Please wait a moment before trying again.';
    case 503:
      return 'The chatbot is temporarily unavailable. Please try again.';
    case 504:
      return 'The response is taking longer than expected. Please try again.';
    default:
      return 'An error occurred. Please try again.';
  }
}

/**
 * Check if an error is retryable
 * @param statusCode - HTTP status code
 * @returns True if error is retryable
 */
export function isRetryableError(statusCode: number): boolean {
  return statusCode === 429 || statusCode === 503 || statusCode === 504;
}

/**
 * Format confidence score as percentage
 * @param confidence - Confidence score (0.0 to 1.0)
 * @returns Percentage string (e.g., "89%")
 */
export function formatConfidence(confidence: number): string {
  return `${Math.round(confidence * 100)}%`;
}
