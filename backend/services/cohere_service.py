"""
Cohere client setup and service for embedding operations
"""
import os
import logging
import time
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv
import cohere

# Load environment variables
load_dotenv()

# Configure logging
logger = logging.getLogger(__name__)


class CohereService:
    """
    Service for interacting with Cohere API for embeddings
    """

    def __init__(self):
        """
        Initialize the Cohere client with configuration from environment
        """
        api_key = os.getenv("COHERE_API_KEY")

        if not api_key:
            raise ValueError("COHERE_API_KEY must be set in environment variables")

        # Initialize Cohere client
        self.client = cohere.Client(api_key)
        self.model = "embed-multilingual-v3.0"  # Using the same model as in the existing pipeline
        logger.info("CohereService initialized")

    def health_check(self) -> bool:
        """
        Check if the Cohere service is available and accessible

        Returns:
            True if service is healthy, False otherwise
        """
        try:
            # Try to generate a simple embedding to verify the connection
            test_embedding = self.generate_embeddings(["test"])
            if test_embedding and len(test_embedding) > 0:
                logger.info("Cohere connection healthy")
                return True
            return False
        except Exception as e:
            logger.error(f"Cohere health check failed: {str(e)}")
            return False

    def generate_embeddings(self, texts: List[str], batch_size: int = 96) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere API

        Args:
            texts: List of texts to generate embeddings for
            batch_size: Size of batches to process (Cohere's max is 96)

        Returns:
            List of embeddings (each embedding is a list of floats)
        """
        all_embeddings = []

        # Process in batches to respect API limits
        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]

            try:
                # Generate embeddings using Cohere
                response = self.client.embed(
                    texts=batch,
                    model=self.model,
                    input_type="search_query"  # Using search_query for queries, search_document for stored content
                )

                # Extract embeddings from response
                batch_embeddings = response.embeddings
                all_embeddings.extend(batch_embeddings)

                logger.info(f"Generated embeddings for batch {i//batch_size + 1}/{(len(texts)-1)//batch_size + 1}")

            except Exception as e:
                logger.error(f"Error generating embeddings for batch: {str(e)}")
                # Return partial results if available, or raise the error
                raise e

        return all_embeddings

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text

        Args:
            text: Text to generate embedding for

        Returns:
            Embedding as a list of floats
        """
        try:
            embeddings = self.generate_embeddings([text])
            return embeddings[0] if embeddings else []
        except Exception as e:
            logger.error(f"Error generating embedding for text: {str(e)}")
            raise e

    def validate_connection(self) -> bool:
        """
        Validate that the Cohere connection is working properly

        Returns:
            True if connection is valid, False otherwise
        """
        try:
            # Try to generate a simple embedding to verify the connection
            test_embedding = self.embed_text("test")
            if test_embedding and len(test_embedding) > 0:
                logger.info("Cohere connection is valid")
                return True
            return False
        except Exception as e:
            logger.error(f"Failed to validate Cohere connection: {str(e)}")
            return False

    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the embedding model

        Returns:
            Model information
        """
        return {
            "model": self.model,
            "vector_size": 1024,  # Cohere's multilingual-v3.0 returns 1024-dim vectors
            "input_type": "search_query for queries, search_document for stored content"
        }