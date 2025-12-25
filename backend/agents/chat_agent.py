"""
OpenAI Assistant controller class
"""
import os
import logging
from typing import Dict, Any, Optional, List
from datetime import datetime

import openai
from dotenv import load_dotenv

from models.query import UserQuery
from models.response import GeneratedResponse
from models.citation import Citation
from models.context import RetrievedContext
from .retrieval_tool import RetrievalTool
from .response_tool import ResponseTool

# Load environment variables
load_dotenv()

# Configure logging
logger = logging.getLogger(__name__)

# Set OpenAI API key or OpenRouter configuration
openai_api_key = os.getenv("OPENAI_API_KEY")
openrouter_api_key = os.getenv("OPENROUTER_API_KEY")
openrouter_base_url = os.getenv("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1")
openrouter_model = os.getenv("OPENROUTER_MODEL", "nex-agi/deepseek-v3.1-nex-n1:free")

# Configure OpenAI client to use OpenRouter if OpenAI key is not available
if openrouter_api_key:
    client = openai.OpenAI(
        api_key=openrouter_api_key,
        base_url=openrouter_base_url
    )
    assistant_model = openrouter_model
    logger.info("Using OpenRouter API for chat completions")
elif openai_api_key:
    client = openai.OpenAI(api_key=openai_api_key)
    assistant_model = os.getenv("OPENAI_MODEL", "gpt-4-turbo")
    logger.info("Using OpenAI API for chat completions")
else:
    raise ValueError("Either OPENAI_API_KEY or OPENROUTER_API_KEY must be set")


class ChatAgent:
    """
    OpenAI Assistant controller class for managing RAG chatbot functionality
    """

    def __init__(self):
        """
        Initialize the Chat Agent with OpenAI client and configuration
        """
        global client, assistant_model
        self.client = client
        self.assistant_instructions = """
        You are an AI assistant specialized in answering questions about Physical AI and Humanoid Robotics.
        You will receive context from a knowledge base and should provide accurate, helpful answers based on that context.
        Always cite your sources using the provided citation information.
        If the context doesn't contain information to answer the question, politely say so and suggest the user try a different query.
        Maintain a professional and informative tone.
        """
        self.assistant_model = assistant_model
        self.retrieval_tool = RetrievalTool()
        self.response_tool = ResponseTool()
        logger.info("ChatAgent initialized successfully")

    def process_query(
        self,
        user_query: UserQuery,
        retrieved_context: List[RetrievedContext],
        confidence_threshold: float = 0.7
    ) -> GeneratedResponse:
        """
        Process a user query with retrieved context and return a generated response

        Args:
            user_query: The user's query with session information
            retrieved_context: List of context chunks retrieved from the knowledge base
            confidence_threshold: Minimum confidence score for valid responses

        Returns:
            GeneratedResponse with the answer and citations
        """
        try:
            # If no context provided, retrieve it using the tool
            if not retrieved_context:
                retrieved_context = self.retrieval_tool.retrieve_context(user_query.query_text)

            # Prepare context for the assistant
            context_text = "\n\n".join([chunk.content for chunk in retrieved_context])

            # Create messages for the assistant
            messages = [
                {"role": "system", "content": self.assistant_instructions},
                {"role": "user", "content": f"Context: {context_text}\n\nQuestion: {user_query.query_text}"}
            ]

            # Generate response using OpenAI API
            response = self.client.chat.completions.create(
                model=self.assistant_model,
                messages=messages,
                temperature=0.3,  # Lower temperature for more consistent, factual responses
                max_tokens=1000
            )

            # Extract the response text
            response_text = response.choices[0].message.content

            # Calculate a confidence score based on response quality and context relevance
            # This is a simplified approach - in a real implementation, this would be more sophisticated
            confidence_score = self._calculate_confidence_score(
                response_text,
                retrieved_context,
                user_query.query_text
            )

            # Create citations from the retrieved context
            citations = self._create_citations(retrieved_context)

            # Check if confidence is above threshold
            if confidence_score < confidence_threshold:
                response_text = "I couldn't find sufficient information in the knowledge base to answer your question confidently. Please try rephrasing your query."
                confidence_score = 0.0

            # Create and return the generated response
            generated_response = GeneratedResponse(
                response_text=response_text,
                confidence_score=confidence_score,
                citations=citations,
                session_id=user_query.session_id
            )

            logger.info(f"Processed query for session {user_query.session_id} with confidence {confidence_score}")
            return generated_response

        except Exception as e:
            logger.error(f"Error processing query: {str(e)}")
            # Return a fallback response
            return GeneratedResponse(
                response_text="I'm sorry, but I encountered an error while processing your request. Please try again.",
                confidence_score=0.0,
                citations=[],
                session_id=user_query.session_id
            )

    def retrieve_and_process_query(
        self,
        user_query: UserQuery,
        confidence_threshold: float = 0.7
    ) -> GeneratedResponse:
        """
        Retrieve context and process a user query in one step

        Args:
            user_query: The user's query with session information
            confidence_threshold: Minimum confidence score for valid responses

        Returns:
            GeneratedResponse with the answer and citations
        """
        import time
        start_time = time.time()

        try:
            # Retrieve context using the tool
            retrieved_context = self.retrieval_tool.retrieve_context(user_query.query_text)

            # Process the query with the retrieved context
            response = self.process_query(user_query, retrieved_context, confidence_threshold)

            # Apply response tool for additional processing/validation
            validated_response = self.response_tool.apply_confidence_threshold(response, confidence_threshold)

            # Track response time
            response_time = time.time() - start_time
            logger.info(f"Processed query in {response_time:.2f}s for session {user_query.session_id}")

            # Check if response time exceeds target (5 seconds)
            if response_time > 5.0:
                logger.warning(f"Response time exceeded 5-second target: {response_time:.2f}s")

            return validated_response

        except Exception as e:
            logger.error(f"Error retrieving and processing query: {str(e)}")
            # Track error response time
            response_time = time.time() - start_time
            logger.info(f"Error response time: {response_time:.2f}s for session {user_query.session_id}")

            # Return a fallback response
            return GeneratedResponse(
                response_text="I'm sorry, but I encountered an error while processing your request. Please try again.",
                confidence_score=0.0,
                citations=[],
                session_id=user_query.session_id
            )

    def _calculate_confidence_score(
        self,
        response_text: str,
        retrieved_context: List[RetrievedContext],
        query_text: str
    ) -> float:
        """
        Calculate a confidence score based on response quality and context relevance

        Args:
            response_text: The generated response text
            retrieved_context: List of context chunks used
            query_text: The original user query

        Returns:
            Confidence score between 0.0 and 1.0
        """
        # Simple heuristic for confidence calculation
        # In a real implementation, this would be more sophisticated
        if not response_text or "I don't know" in response_text or "not mentioned" in response_text:
            return 0.1  # Very low confidence if the response indicates lack of knowledge

        # Calculate average relevance of the context used
        if retrieved_context:
            avg_relevance = sum(chunk.relevance_score for chunk in retrieved_context) / len(retrieved_context)
            # Adjust based on how well the response seems to address the query
            # This is a simplified approach - real implementation would use more sophisticated NLP
            return min(avg_relevance, 0.95)  # Cap at 0.95 to account for uncertainty
        else:
            return 0.1  # Low confidence if no context was provided

    def _create_citations(self, retrieved_context: List[RetrievedContext]) -> List[Citation]:
        """
        Create citation objects from retrieved context

        Args:
            retrieved_context: List of context chunks with metadata

        Returns:
            List of Citation objects
        """
        citations = []
        for chunk in retrieved_context:
            citation = Citation(
                source_url=chunk.source_url,
                chapter=chunk.chapter,
                section=chunk.section,
                content_snippet=chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content,
                relevance_score=chunk.relevance_score
            )
            citations.append(citation)

        return citations

    def validate_query(self, query_text: str) -> bool:
        """
        Validate the user query for safety and appropriateness

        Args:
            query_text: The user's query text

        Returns:
            True if query is valid, False otherwise
        """
        # Basic validation - in a real implementation, this would include more sophisticated checks
        if not query_text or len(query_text.strip()) < 3:
            return False

        # Check for potentially harmful content (simplified check)
        harmful_keywords = ["injection", "exploit", "attack", "hack"]
        if any(keyword in query_text.lower() for keyword in harmful_keywords):
            return False

        return True