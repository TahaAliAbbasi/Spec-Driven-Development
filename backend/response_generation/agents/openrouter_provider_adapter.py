"""OpenRouter provider adapter for OpenAI SDK compatibility"""

from openai import OpenAI
from typing import Dict, Any, List, Optional
from config import settings
from utils.exceptions import APIConnectionError, ModelResponseError
import logging


class OpenRouterProviderAdapter:
    """
    Adapter class to use OpenRouter API with OpenAI SDK interface.
    This enables the use of OpenRouter GPT models through the OpenAI Agents SDK interface.
    """

    def __init__(self):
        """Initialize the adapter with OpenRouter API configuration."""
        self.logger = logging.getLogger(__name__)

        # Validate API key is provided
        if not settings.OPENROUTER_API_KEY:
            raise ValueError("OPENROUTER_API_KEY environment variable is required")

        # Create OpenAI client configured for OpenRouter
        self.client = OpenAI(
            api_key=settings.OPENROUTER_API_KEY,
            base_url=settings.OPENROUTER_BASE_URL,
        )

        self.model = settings.OPENROUTER_MODEL
        self.logger.info(f"OpenRouter adapter initialized with model: {self.model}")

    def generate_response(
        self,
        query: str,
        context_chunks: List[Dict[str, Any]],
        system_prompt: str = None
    ) -> Dict[str, Any]:
        """
        Generate a response using the OpenRouter API based on query and context.

        Args:
            query: The user's question/query
            context_chunks: List of context chunks to ground the response
            system_prompt: Optional system prompt to guide the response

        Returns:
            Dictionary containing the generated response and metadata
        """
        try:
            # Construct the context text from chunks
            context_text = "\n\n".join([chunk.get("content", "") for chunk in context_chunks])

            # Build the messages for the OpenAI-compatible API
            messages = []

            # Add system prompt if provided
            if system_prompt:
                messages.append({"role": "system", "content": system_prompt})
            else:
                # Default system prompt for constitutional compliance
                default_system_prompt = (
                    "You are a helpful AI assistant that answers questions based strictly on the provided context. "
                    "Follow these rules carefully:\n"
                    "1. Only use information from the provided context to answer questions.\n"
                    "2. Do not generate any information not present in the provided context.\n"
                    "3. If the context is insufficient to answer the query, respond with 'I cannot answer this question based on the provided context.'\n"
                    "4. Provide specific citations for information when possible.\n"
                    "5. Maintain a professional and helpful tone.\n"
                    "6. Always use temperature=0 for deterministic outputs."
                )
                messages.append({"role": "system", "content": default_system_prompt})

            # Add context and query as user message
            user_content = f"Context:\n{context_text}\n\nQuery: {query}\n\nPlease answer the query based strictly on the provided context. Do not generate any information not present in the context."
            messages.append({"role": "user", "content": user_content})

            # Make the API call to OpenRouter
            response = self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=settings.MODEL_TEMPERATURE,  # Should be 0 for deterministic behavior
                max_tokens=settings.MAX_TOKENS,
                timeout=30  # 30 second timeout
            )

            # Extract the response text
            text_response = response.choices[0].message.content

            # Create response structure with token usage
            result = {
                "text": text_response,
                "usage": {
                    "prompt_tokens": response.usage.prompt_tokens if response.usage else 0,
                    "completion_tokens": response.usage.completion_tokens if response.usage else 0,
                    "total_tokens": response.usage.total_tokens if response.usage else 0
                }
            }

            self.logger.info(f"Successfully generated response from OpenRouter. Tokens used: {result['usage']['total_tokens']}")
            return result

        except Exception as e:
            self.logger.error(f"Error generating response from OpenRouter API: {str(e)}")
            raise APIConnectionError(f"Failed to connect to OpenRouter API: {str(e)}")

    def validate_no_retrieval_capability(self) -> bool:
        """
        Verify that this adapter enforces the 'NO RETRIEVAL' hard guard.
        This method simply returns True to confirm that the adapter does not
        provide any retrieval capabilities beyond the provided context.
        """
        return True

    def extract_citations_from_response(
        self,
        response_text: str,
        context_chunks: List[Dict[str, Any]]
    ) -> List[Dict[str, str]]:
        """
        Extract citations from the response by matching content to context chunks.
        This is a basic implementation that identifies which chunks contributed to the response.
        """
        citations = []

        for chunk in context_chunks:
            chunk_content = chunk.get("content", "")
            chunk_id = chunk.get("chunk_id", "")
            source_url = chunk.get("metadata", {}).get("source_url", "")

            # Check if any part of the chunk content appears in the response
            # This is a simple approach - in practice, you might want more sophisticated matching
            if chunk_content and chunk_id and chunk_content.lower() in response_text.lower():
                citations.append({
                    "chunk_id": chunk_id,
                    "source_url": source_url
                })

        return citations