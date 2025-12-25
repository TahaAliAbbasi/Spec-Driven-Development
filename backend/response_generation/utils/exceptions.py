"""Custom exception classes for response generation errors"""


class ResponseGenerationError(Exception):
    """Base exception for response generation errors"""
    pass


class ConstitutionalViolationError(ResponseGenerationError):
    """Raised when a constitutional requirement is violated (e.g., hallucination detected)"""
    pass


class ContextInsufficientError(ResponseGenerationError):
    """Raised when the provided context is insufficient to answer the query"""
    pass


class SelectedTextOnlyViolationError(ResponseGenerationError):
    """Raised when selected-text-only mode requirements are not met"""
    pass


class AgentProcessingError(ResponseGenerationError):
    """Raised when there's an error during agent processing"""
    pass


class APIConnectionError(ResponseGenerationError):
    """Raised when there's an error connecting to external APIs"""
    pass


class ModelResponseError(ResponseGenerationError):
    """Raised when the model returns an invalid or unexpected response"""
    pass