// Physical-AI-and-Humanoid-Robotics/src/components/QueryInput.tsx
import React, { useState, useEffect, FormEvent, ChangeEvent } from 'react';
import { validateQuery, sanitizeQuery } from '../utils/validation';
import { COMPONENT_DEFAULTS } from '../config/constants';

interface QueryInputProps {
  onSubmit: (query: string) => Promise<void>;
  placeholder?: string;
  disabled?: boolean;
}

const QueryInput: React.FC<QueryInputProps> = ({
  onSubmit,
  placeholder = COMPONENT_DEFAULTS.QUERY_INPUT_PLACEHOLDER,
  disabled = false,
}) => {
  const [query, setQuery] = useState('');
  const [error, setError] = useState('');
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [charCount, setCharCount] = useState(0);

  useEffect(() => {
    setCharCount(query.length);
  }, [query]);

  const handleSubmit = async (e: FormEvent<HTMLFormElement>) => {
    e.preventDefault();
    if (disabled || isSubmitting) return;

    const validation = validateQuery(query);
    if (!validation.isValid) {
      setError(validation.error ?? '');
      return;
    }

    setError('');
    setIsSubmitting(true);

    try {
      const sanitizedQuery = sanitizeQuery(query);
      await onSubmit(sanitizedQuery);
      setQuery('');
    } catch (err: any) {
      setError(err?.message || COMPONENT_DEFAULTS.ERROR_MESSAGES.API_ERROR);
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleQueryChange = (e: ChangeEvent<HTMLTextAreaElement>) => {
    const newQuery = e.target.value;
    if (newQuery.length > 1000) return;
    setQuery(newQuery);

    if (error) {
      const validation = validateQuery(newQuery);
      if (validation.isValid) setError('');
    }
  };

  const getCharCountClass = () => (charCount > 900 ? 'char-count-warning' : 'char-count-normal');

  return (
    <div
      className="query-input-container"
      role="region"
      aria-labelledby="query-input-title"
    >
      <div id="query-input-title" className="sr-only">
        Query Input Section
      </div>
      <form onSubmit={handleSubmit} className="query-form" role="form">
        <div className="query-input-wrapper">
          <textarea
            id="query-textarea"
            value={query}
            onChange={handleQueryChange}
            placeholder={placeholder}
            disabled={disabled || isSubmitting}
            className={`query-textarea ${error ? 'error' : ''}`}
            rows={4}
            maxLength={1000}
            aria-label="Enter your question about the Physical AI and Humanoid Robotics book"
            aria-describedby={error ? 'query-error-message' : 'query-char-count'}
            aria-invalid={!!error}
          />
          <div className="input-footer">
            <div
              id="query-char-count"
              className={`char-count ${getCharCountClass()}`}
              aria-live="polite"
            >
              {charCount}/1000 characters
            </div>
            {error && (
              <div id="query-error-message" className="query-error-message" role="alert">
                {error}
              </div>
            )}
          </div>
        </div>
        <button
          type="submit"
          disabled={disabled || isSubmitting || !query.trim()}
          className="query-submit-button"
          aria-label="Submit your query"
        >
          {isSubmitting ? COMPONENT_DEFAULTS.LOADING_TEXT : COMPONENT_DEFAULTS.SUBMIT_BUTTON_TEXT}
        </button>
      </form>
    </div>
  );
};

export default QueryInput;
