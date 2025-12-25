"""
Response service with confidence thresholding logic
"""
import os
import logging
from typing import Dict, Any, List, Optional
from datetime import datetime
from dotenv import load_dotenv

from models.response import GeneratedResponse
from models.citation import Citation
from models.context import RetrievedContext

# Load environment variables
load_dotenv()

# Configure logging
logger = logging.getLogger(__name__)


class ResponseService:
    """
    Service for handling response generation with confidence thresholding
    """

    def __init__(self):
        """
        Initialize the response service
        """
        self.default_confidence_threshold = float(os.getenv("DEFAULT_CONFIDENCE_THRESHOLD", "0.7"))
        self.low_confidence_message = os.getenv(
            "LOW_CONFIDENCE_MESSAGE",
            "I couldn't find sufficient information in the knowledge base to answer your question confidently. Please try rephrasing your query."
        )
        logger.info(f"ResponseService initialized with threshold: {self.default_confidence_threshold}")

    def generate_response(
        self,
        query: str,
        context_chunks: List[RetrievedContext],
        threshold: Optional[float] = None
    ) -> GeneratedResponse:
        """
        Generate a response based on query and context, applying confidence thresholding

        Args:
            query: The user's query
            context_chunks: Retrieved context chunks to base the response on
            threshold: Confidence threshold (uses default if not provided)

        Returns:
            GeneratedResponse with appropriate confidence level
        """
        if threshold is None:
            threshold = self.default_confidence_threshold

        # Calculate the response and confidence
        response_text, confidence = self._generate_response_with_confidence(query, context_chunks)

        # Apply confidence threshold
        if confidence < threshold:
            response_text = self.low_confidence_message
            confidence = 0.0

        # Create citations from context
        citations = self._create_citations_from_context(context_chunks)

        # Create and return the response
        return GeneratedResponse(
            response_text=response_text,
            confidence_score=confidence,
            citations=citations,
            timestamp=datetime.now()
        )

    def _generate_response_with_confidence(self, query: str, context_chunks: List[RetrievedContext]) -> tuple[str, float]:
        """
        Generate response text and calculate confidence score

        Args:
            query: The user's query
            context_chunks: Retrieved context chunks

        Returns:
            Tuple of (response_text, confidence_score)
        """
        # In a real implementation, this would call an LLM or other response generation system
        # For now, we'll simulate the response generation

        if not context_chunks:
            return "I couldn't find any relevant information in the knowledge base to answer your question.", 0.1

        # Calculate an average relevance score from the context
        avg_relevance = sum(chunk.relevance_score for chunk in context_chunks) / len(context_chunks)

        # Simulate generating a response based on context
        response_text = self._synthesize_response(query, context_chunks)

        # Calculate confidence based on average relevance and number of context chunks
        confidence = min(avg_relevance, 0.95)  # Cap at 0.95 to account for uncertainty

        return response_text, confidence

    def _synthesize_response(self, query: str, context_chunks: List[RetrievedContext]) -> str:
        """
        Synthesize a response based on query and context chunks

        Args:
            query: The user's query
            context_chunks: Retrieved context chunks

        Returns:
            Generated response text
        """
        # In a real implementation, this would use an LLM to synthesize the response
        # For now, we'll create a simple synthesized response
        context_text = " ".join([chunk.content for chunk in context_chunks[:2]])  # Use first 2 chunks

        # Simple response synthesis (in real implementation, use an LLM)
        response = f"Based on the information available: {context_text[:500]}..."  # Truncate to 500 chars
        return response

    def _create_citations_from_context(self, context_chunks: List[RetrievedContext]) -> List[Citation]:
        """
        Create citation objects from retrieved context chunks according to specification

        Args:
            context_chunks: Retrieved context chunks

        Returns:
            List of Citation objects with standardized format and source links
        """
        citations = []
        for chunk in context_chunks:
            citation = Citation(
                source_url=chunk.source_url,
                chapter=chunk.chapter,
                section=chunk.section,
                content_snippet=chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content,
                relevance_score=chunk.relevance_score
            )
            citations.append(citation)

        return citations

    def format_citations_for_response(self, citations: List[Citation]) -> str:
        """
        Format citations in a standardized format with source links for response display

        Args:
            citations: List of citations to format

        Returns:
            Formatted string of citations
        """
        if not citations:
            return ""

        formatted_citations = ["\n\n**Sources:**"]
        for i, citation in enumerate(citations, 1):
            # Format each citation with proper structure and links
            formatted_citation = (
                f"{i}. [{citation.chapter or 'Reference'} - {citation.section or 'Section'}]({citation.source_url}) "
                f"- Relevance: {citation.relevance_score:.2f}" if citation.relevance_score else ""
            )
            formatted_citations.append(formatted_citation)

        return "\n".join(formatted_citations)

    def add_citations_to_response(self, response_text: str, citations: List[Citation]) -> str:
        """
        Add formatted citations to a response text

        Args:
            response_text: Original response text
            citations: Citations to add to the response

        Returns:
            Response text with properly formatted citations appended
        """
        if not citations:
            return response_text

        formatted_citations = self.format_citations_for_response(citations)
        return f"{response_text}{formatted_citations}"

    def validate_response_quality(self, response: GeneratedResponse, threshold: float) -> Dict[str, Any]:
        """
        Validate that the response meets quality requirements

        Args:
            response: The generated response to validate
            threshold: Minimum confidence threshold

        Returns:
            Validation results with pass/fail status and details
        """
        validation_result = {
            "pass": True,
            "issues": [],
            "confidence_score": response.confidence_score,
            "threshold": threshold
        }

        # Check confidence threshold
        if response.confidence_score < threshold:
            validation_result["pass"] = False
            validation_result["issues"].append(f"Confidence score {response.confidence_score} is below threshold {threshold}")

        # Check that response has content
        if not response.response_text or len(response.response_text.strip()) == 0:
            validation_result["pass"] = False
            validation_result["issues"].append("Response text is empty")

        # Check that response has citations if confidence is high
        if response.confidence_score > 0.5 and len(response.citations) == 0:
            validation_result["pass"] = False
            validation_result["issues"].append("High confidence response should have citations")

        return validation_result

    def apply_confidence_threshold(
        self,
        response: GeneratedResponse,
        threshold: Optional[float] = None
    ) -> GeneratedResponse:
        """
        Apply confidence threshold to an existing response

        Args:
            response: The response to apply threshold to
            threshold: Confidence threshold (uses default if not provided)

        Returns:
            Response with threshold applied
        """
        if threshold is None:
            threshold = self.default_confidence_threshold

        if response.confidence_score < threshold:
            # Modify the response to indicate low confidence
            modified_response = GeneratedResponse(
                response_text=self.low_confidence_message,
                confidence_score=0.0,
                citations=[],
                timestamp=response.timestamp,
                session_id=response.session_id
            )
            logger.info(f"Applied confidence threshold, response confidence was below {threshold}")
            return modified_response

        return response