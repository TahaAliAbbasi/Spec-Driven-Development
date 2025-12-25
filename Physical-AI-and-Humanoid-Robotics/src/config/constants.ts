// Physical-AI-and-Humanoid-Robotics/src/config/constants.ts
// Constants and configuration module

/**
 * API configuration
 */
export const API_CONFIG = {
  BASE_URL:
    process.env.REACT_APP_API_BASE_URL ?? 'http://localhost:8000',
  TIMEOUT: 30_000, // 30 seconds
  RETRY_ATTEMPTS: 3,
  RATE_LIMIT_DELAY: 1_000 // 1 second debounce
} as const;

/**
 * Query modes (string literal safe)
 */
export const QUERY_MODES = {
  GLOBAL: 'global',
  SELECTED_TEXT_ONLY: 'selected_text_only'
} as const;

/**
 * Query mode type (derived from QUERY_MODES)
 */
export type QueryMode =
  typeof QUERY_MODES[keyof typeof QUERY_MODES];

/**
 * Backend response status
 */
export const RESPONSE_STATUS = {
  ANSWERED: 'ANSWERED',
  INSUFFICIENT_CONTEXT: 'INSUFFICIENT_CONTEXT',
  REFUSED: 'REFUSED'
} as const;

/**
 * Response status type
 */
export type ResponseStatus =
  typeof RESPONSE_STATUS[keyof typeof RESPONSE_STATUS];

/**
 * UI component defaults
 */
export const COMPONENT_DEFAULTS = {
  QUERY_INPUT_PLACEHOLDER:
    'Ask a question about the Physical AI and Humanoid Robotics book...',
  SUBMIT_BUTTON_TEXT: 'Submit Query',
  LOADING_TEXT: 'Processing your query...',
  ERROR_MESSAGES: {
    EMPTY_QUERY: 'Please enter a question',
    INVALID_QUERY: 'Query must be between 1 and 1000 characters',
    API_ERROR: 'Error processing your query. Please try again.',
    INSUFFICIENT_CONTEXT:
      'The provided context is insufficient to answer this question.',
    REFUSED:
      'The query was refused based on constitutional compliance rules.'
  }
} as const;

export default {
  API_CONFIG,
  QUERY_MODES,
  RESPONSE_STATUS,
  COMPONENT_DEFAULTS
};
