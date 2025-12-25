// Physical-AI-and-Humanoid-Robotics/src/services/api.ts
// API service module for connecting to Phase 3 backend

import { API_CONFIG } from '../config/constants';
import type { ApiResponse, QueryContext } from '../types';

let lastRequestTime = 0;

/**
 * Debounce function to limit API requests
 * @param func - Function to debounce
 * @param delay - Delay in milliseconds
 * @returns Debounced function
 */
const debounce = <T extends (...args: unknown[]) => void>(
  func: T,
  delay: number
): ((...args: Parameters<T>) => void) => {
  let timeoutId: ReturnType<typeof setTimeout>;

  return (...args: Parameters<T>) => {
    clearTimeout(timeoutId);
    timeoutId = setTimeout(() => func(...args), delay);
  };
};

/**
 * Submit a query to the Phase 3 backend API with rate limiting
 * @param query - The user's question/query
 * @param mode - Query mode ('global' or 'selected_text_only')
 * @param context - Additional context for the query
 * @returns Response from the backend API
 */
export const submitQuery = async (
  query: string,
  mode: 'global' | 'selected_text_only' = 'global',
  context: QueryContext = {}
): Promise<ApiResponse> => {
  const now = Date.now();
  const timeSinceLastRequest = now - lastRequestTime;

  // Basic rate limiting
  if (timeSinceLastRequest < API_CONFIG.RATE_LIMIT_DELAY) {
    await new Promise<void>(resolve =>
      setTimeout(resolve, API_CONFIG.RATE_LIMIT_DELAY - timeSinceLastRequest)
    );
  }

  lastRequestTime = Date.now();

  try {
    const controller = new AbortController();
    const timeoutId = setTimeout(
      () => controller.abort(),
      API_CONFIG.TIMEOUT
    );

    const response = await fetch(`${API_CONFIG.BASE_URL}/api/answer`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        Accept: 'application/json'
      },
      body: JSON.stringify({
        query,
        mode,
        context
      }),
      signal: controller.signal
    });

    clearTimeout(timeoutId);

    if (!response.ok) {
      if (response.status === 429) {
        throw new Error(
          'Rate limit exceeded. Please wait before making another request.'
        );
      } else if (response.status >= 500) {
        throw new Error(
          `Server error (${response.status}). The backend service may be temporarily unavailable.`
        );
      } else if (response.status === 400) {
        throw new Error(
          'Bad request. Please check your query and try again.'
        );
      } else {
        throw new Error(`API request failed with status ${response.status}`);
      }
    }

    const result: ApiResponse = await response.json();
    return result;
  } catch (error: unknown) {
    if (error instanceof DOMException && error.name === 'AbortError') {
      throw new Error(
        'Request timeout. The server took too long to respond.'
      );
    }

    console.error('Error submitting query:', error);
    throw error;
  }
};

/**
 * Health check for the backend API
 * @returns Whether the API is available
 */
export const checkApiHealth = async (): Promise<boolean> => {
  try {
    const controller = new AbortController();
    const timeoutId = setTimeout(
      () => controller.abort(),
      API_CONFIG.TIMEOUT / 2
    );

    const response = await fetch(
      `${API_CONFIG.BASE_URL}/api/health/answer`,
      {
        signal: controller.signal
      }
    );

    clearTimeout(timeoutId);
    return response.ok;
  } catch (error) {
    console.error('API health check failed:', error);
    return false;
  }
};

export default {
  submitQuery,
  checkApiHealth
};
