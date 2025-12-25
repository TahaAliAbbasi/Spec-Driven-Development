"""
Agent tool for response generation
"""
import os
import logging
from typing import Dict, Any, List
from dotenv import load_dotenv

from models.query import UserQuery
from models.response import GeneratedResponse
from models.context import RetrievedContext
from services.response_service import ResponseService

# Load environment variables
load_dotenv()

# Configure logging
logger = logging.getLogger(__name__)


class ResponseTool:
    """
    Agent tool for generating responses based on context
    """

    def __init__(self):
        """
        Initialize the response generation tool
        """
        self.response_service = ResponseService()
        logger.info("ResponseTool initialized")

    def generate_response(
        self,
        user_query: UserQuery,
        context_chunks: List[RetrievedContext],
        threshold: float = 0.7
    ) -> GeneratedResponse:
        """
        Generate a response based on user query and context

        Args:
            user_query: The user's query with session information
            context_chunks: Retrieved context chunks to base the response on
            threshold: Confidence threshold for valid responses

        Returns:
            GeneratedResponse object
        """
        try:
            # Generate response using the response service
            response = self.response_service.generate_response(
                user_query.query_text,
                context_chunks,
                threshold
            )

            # Set session ID from user query
            response.session_id = user_query.session_id

            logger.info(f"Generated response with confidence: {response.confidence_score}")
            return response

        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            # Return a fallback response
            return GeneratedResponse(
                response_text="I'm sorry, but I encountered an error while generating a response. Please try again.",
                confidence_score=0.0,
                citations=[],
                session_id=user_query.session_id
            )

    def validate_response_quality(self, response: GeneratedResponse, threshold: float = 0.7) -> Dict[str, Any]:
        """
        Validate the quality of a generated response

        Args:
            response: The response to validate
            threshold: Minimum confidence threshold

        Returns:
            Dictionary with validation results
        """
        return self.response_service.validate_response_quality(response, threshold)

    def apply_confidence_threshold(
        self,
        response: GeneratedResponse,
        threshold: float = 0.7
    ) -> GeneratedResponse:
        """
        Apply confidence threshold to a response

        Args:
            response: The response to apply threshold to
            threshold: Confidence threshold to apply

        Returns:
            Response with threshold applied
        """
        return self.response_service.apply_confidence_threshold(response, threshold)

    def get_response_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the response service

        Returns:
            Dictionary with response service statistics
        """
        return {
            "default_threshold": self.response_service.default_confidence_threshold,
            "low_confidence_message": self.response_service.low_confidence_message
        }