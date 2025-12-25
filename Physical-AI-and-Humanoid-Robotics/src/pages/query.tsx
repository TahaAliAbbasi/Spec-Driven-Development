// Physical-AI-and-Humanoid-Robotics/src/pages/query.tsx
// Interactive query page component

import React, { useState, useCallback } from 'react';
import QueryInput from '../components/QueryInput';
import ResponseViewer from '../components/ResponseViewer';
import ErrorNotifier from '../components/ErrorNotifier';
import { submitQuery } from '../services/api';
import { QUERY_MODES, API_CONFIG } from '../config/constants';
import type { ApiResponse } from '../types';
import '../css/query-page.css';

interface Notification {
  message: string;
  type: 'success' | 'error';
}

const QueryPage: React.FC = () => {
  const [response, setResponse] = useState<ApiResponse | null>(null);
  const [loading, setLoading] = useState<boolean>(false);
  const [error, setError] = useState<string | null>(null);
  const [notification, setNotification] = useState<Notification | null>(null);

  /**
   * Debounce function to prevent rapid successive queries
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

  const handleQuerySubmit = async (queryText: string): Promise<void> => {
    setLoading(true);
    setError(null);
    setResponse(null);
    setNotification(null);

    try {
      // Submit query to Phase 3 backend
      const result = await submitQuery(queryText, QUERY_MODES.GLOBAL);
      setResponse(result);

      // Show success notification if response was positive
      if (result.status === 'ANSWERED') {
        setNotification({
          message: 'Query processed successfully!',
          type: 'success'
        });
      }
    } catch (err: unknown) {
      const errorMessage =
        err instanceof Error ? err.message : 'Failed to process query';

      setError(errorMessage);
      setNotification({
        message: errorMessage,
        type: 'error'
      });

      console.error('Query submission error:', err);
    } finally {
      setLoading(false);
    }
  };

  // Apply debouncing to prevent API spam
  const debouncedHandleQuerySubmit = useCallback(
    debounce(handleQuerySubmit, API_CONFIG.RATE_LIMIT_DELAY),
    []
  );

  const clearNotification = (): void => {
    setNotification(null);
  };

  return (
    <div className="query-page">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <h1>Interactive Query System</h1>
            <p>
              Ask questions about the Physical AI and Humanoid Robotics book
              content. The AI will provide responses based on the book&apos;s
              context with proper citations.
            </p>

            {notification && (
              <ErrorNotifier
                message={notification.message}
                type={notification.type}
                visible={true}
                onClose={clearNotification}
              />
            )}

            <div className="query-section">
              <QueryInput
                onSubmit={debouncedHandleQuerySubmit}
                disabled={loading}
              />
            </div>

            <div className="response-section">
              <ResponseViewer
                response={response}
                loading={loading}
                error={error}
              />
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default QueryPage;
