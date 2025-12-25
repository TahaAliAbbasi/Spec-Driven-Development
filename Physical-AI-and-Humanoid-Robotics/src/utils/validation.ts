// Physical-AI-and-Humanoid-Robotics/src/utils/validation.ts
// Utility functions for input validation

/**
 * Generic validation result interface
 */
export interface ValidationResult {
  isValid: boolean;
  error: string | null;
}

/**
 * Backend response interface
 */
export interface BackendResponse {
  status: 'ANSWERED' | 'INSUFFICIENT_CONTEXT' | 'REFUSED';
  answer?: string;
  [key: string]: unknown;
}

/**
 * Validates a query string
 * @param query - The query to validate
 * @returns Validation result with isValid and error message
 */
export const validateQuery = (query: string): ValidationResult => {
  if (!query || query.trim().length === 0) {
    return {
      isValid: false,
      error: 'Query cannot be empty'
    };
  }

  if (query.length > 1000) {
    return {
      isValid: false,
      error: 'Query must be less than 1000 characters'
    };
  }

  // Basic XSS prevention - check for common script tags
  const xssPatterns: RegExp[] = [
    /<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi,
    /javascript:/gi,
    /vbscript:/gi,
    /onload=/gi,
    /onerror=/gi,
    /onmouseover=/gi,
    /onmouseout=/gi
  ];

  for (const pattern of xssPatterns) {
    if (pattern.test(query)) {
      return {
        isValid: false,
        error: 'Query contains potentially harmful content'
      };
    }
  }

  return {
    isValid: true,
    error: null
  };
};

/**
 * Sanitizes a query string by removing potentially harmful content
 * @param query - The query to sanitize
 * @returns Sanitized query
 */
export const sanitizeQuery = (query: string): string => {
  if (!query) return query;

  const sanitized: string = query
    .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '')
    .replace(/javascript:/gi, '')
    .replace(/vbscript:/gi, '')
    .replace(/onload=/gi, '')
    .replace(/onerror=/gi, '')
    .replace(/onmouseover=/gi, '')
    .replace(/onmouseout=/gi, '');

  return sanitized.trim();
};

/**
 * Validates a response from the backend
 * @param response - The response to validate
 * @returns Validation result with isValid and error message
 */
export const validateResponse = (
  response: BackendResponse | null | undefined
): ValidationResult => {
  if (!response) {
    return {
      isValid: false,
      error: 'Response is null or undefined'
    };
  }

  const requiredFields: Array<keyof BackendResponse> = ['status'];
  for (const field of requiredFields) {
    if (!(field in response)) {
      return {
        isValid: false,
        error: `Response is missing required field: ${field}`
      };
    }
  }

  const validStatuses: BackendResponse['status'][] = [
    'ANSWERED',
    'INSUFFICIENT_CONTEXT',
    'REFUSED'
  ];

  if (!validStatuses.includes(response.status)) {
    return {
      isValid: false,
      error: `Response has invalid status: ${response.status}`
    };
  }

  if (
    response.status === 'ANSWERED' &&
    (!response.answer || typeof response.answer !== 'string')
  ) {
    return {
      isValid: false,
      error: 'ANSWERED response must include a valid answer string'
    };
  }

  return {
    isValid: true,
    error: null
  };
};

export default {
  validateQuery,
  sanitizeQuery,
  validateResponse
};
