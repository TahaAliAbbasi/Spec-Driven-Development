"""
Error handling utilities for the RAG Chatbot Backend
"""
import logging
from typing import Dict, Any, Optional, Union
from datetime import datetime
from enum import Enum

# Configure logging
logger = logging.getLogger(__name__)


class ErrorType(Enum):
    """
    Enum for different types of errors
    """
    VALIDATION_ERROR = "validation_error"
    SERVICE_ERROR = "service_error"
    DATABASE_ERROR = "database_error"
    EXTERNAL_API_ERROR = "external_api_error"
    SECURITY_ERROR = "security_error"
    RATE_LIMIT_ERROR = "rate_limit_error"
    TIMEOUT_ERROR = "timeout_error"
    UNKNOWN_ERROR = "unknown_error"


class ErrorDetails:
    """
    Structure for error details
    """
    def __init__(
        self,
        error_type: ErrorType,
        message: str,
        code: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
        timestamp: Optional[datetime] = None
    ):
        self.error_type = error_type
        self.message = message
        self.code = code or error_type.value
        self.details = details or {}
        self.timestamp = timestamp or datetime.now()

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert error details to dictionary
        """
        return {
            "type": self.error_type.value,
            "message": self.message,
            "code": self.code,
            "details": self.details,
            "timestamp": self.timestamp.isoformat()
        }


class ErrorHandler:
    """
    Utility class for handling errors consistently across the application
    """

    @staticmethod
    def log_error(
        error: Union[Exception, str],
        error_type: ErrorType,
        context: Optional[Dict[str, Any]] = None,
        level: int = logging.ERROR
    ) -> ErrorDetails:
        """
        Log an error with consistent format

        Args:
            error: The error or error message
            error_type: Type of error
            context: Additional context about the error
            level: Logging level

        Returns:
            ErrorDetails object with error information
        """
        message = str(error) if isinstance(error, Exception) else error
        details = context or {}

        error_details = ErrorDetails(
            error_type=error_type,
            message=message,
            details=details
        )

        # Log the error
        logger.log(
            level,
            f"Error [{error_type.value}]: {message}",
            extra={"error_details": error_details.to_dict()}
        )

        return error_details

    @staticmethod
    def handle_validation_error(
        message: str,
        field: Optional[str] = None,
        value: Optional[Any] = None
    ) -> ErrorDetails:
        """
        Handle a validation error

        Args:
            message: Error message
            field: Field that failed validation
            value: Value that failed validation

        Returns:
            ErrorDetails object
        """
        context = {}
        if field:
            context["field"] = field
        if value is not None:
            context["value"] = value

        return ErrorHandler.log_error(
            message,
            ErrorType.VALIDATION_ERROR,
            context
        )

    @staticmethod
    def handle_service_error(
        service_name: str,
        message: str,
        operation: Optional[str] = None
    ) -> ErrorDetails:
        """
        Handle a service error

        Args:
            service_name: Name of the service that failed
            message: Error message
            operation: Operation that failed

        Returns:
            ErrorDetails object
        """
        context = {
            "service": service_name
        }
        if operation:
            context["operation"] = operation

        return ErrorHandler.log_error(
            message,
            ErrorType.SERVICE_ERROR,
            context
        )

    @staticmethod
    def handle_external_api_error(
        api_name: str,
        status_code: Optional[int] = None,
        message: str = "External API error occurred"
    ) -> ErrorDetails:
        """
        Handle an external API error

        Args:
            api_name: Name of the external API
            status_code: HTTP status code if applicable
            message: Error message

        Returns:
            ErrorDetails object
        """
        context = {
            "api": api_name
        }
        if status_code:
            context["status_code"] = status_code

        return ErrorHandler.log_error(
            message,
            ErrorType.EXTERNAL_API_ERROR,
            context
        )

    @staticmethod
    def handle_rate_limit_error(
        limit: Optional[int] = None,
        window: Optional[int] = None,
        message: str = "Rate limit exceeded"
    ) -> ErrorDetails:
        """
        Handle a rate limit error

        Args:
            limit: Rate limit that was exceeded
            window: Time window for the rate limit
            message: Error message

        Returns:
            ErrorDetails object
        """
        context = {}
        if limit:
            context["limit"] = limit
        if window:
            context["window"] = window

        return ErrorHandler.log_error(
            message,
            ErrorType.RATE_LIMIT_ERROR,
            context
        )

    @staticmethod
    def create_error_response(
        error_details: ErrorDetails,
        include_details: bool = False
    ) -> Dict[str, Any]:
        """
        Create a standardized error response for API endpoints

        Args:
            error_details: ErrorDetails object
            include_details: Whether to include detailed error information

        Returns:
            Dictionary with standardized error response
        """
        response = {
            "error": {
                "type": error_details.error_type.value,
                "message": error_details.message,
                "code": error_details.code,
                "timestamp": error_details.timestamp.isoformat()
            }
        }

        if include_details:
            response["error"]["details"] = error_details.details

        return response

    @staticmethod
    def is_retryable_error(error_type: ErrorType) -> bool:
        """
        Determine if an error type is retryable

        Args:
            error_type: Type of error

        Returns:
            True if the error is retryable, False otherwise
        """
        retryable_types = [
            ErrorType.SERVICE_ERROR,
            ErrorType.DATABASE_ERROR,
            ErrorType.EXTERNAL_API_ERROR,
            ErrorType.TIMEOUT_ERROR
        ]
        return error_type in retryable_types

    @staticmethod
    def should_log_error(error_type: ErrorType) -> bool:
        """
        Determine if an error should be logged based on type

        Args:
            error_type: Type of error

        Returns:
            True if the error should be logged, False otherwise
        """
        # All error types should be logged
        return True