"""
Qdrant client setup and service for vector database operations
"""
import os
import logging
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import Distance, VectorParams

# Load environment variables
load_dotenv()

# Configure logging
logger = logging.getLogger(__name__)


class QdrantService:
    """
    Service for interacting with Qdrant vector database
    """

    def __init__(self):
        """
        Initialize the Qdrant client with configuration from environment
        """
        api_key = os.getenv("QDRANT_API_KEY")
        url = os.getenv("QDRANT_URL")
        collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_humanoid_docs_v3")

        if not api_key or not url:
            raise ValueError("QDRANT_API_KEY and QDRANT_URL must be set in environment variables")

        # Initialize Qdrant client
        self.client = QdrantClient(
            url=url,
            api_key=api_key,
        )
        self.collection_name = collection_name
        self.vector_size = 1024  # Cohere's multilingual-v3.0 model returns 1024-dim vectors
        logger.info(f"QdrantService initialized for collection: {self.collection_name}")

    def health_check(self) -> bool:
        """
        Check if the Qdrant service is available and accessible

        Returns:
            True if service is healthy, False otherwise
        """
        try:
            # Try to list collections to verify connection
            collections = self.client.get_collections()
            logger.info(f"Qdrant connection healthy, {len(collections.collections)} collections found")
            return True
        except Exception as e:
            logger.error(f"Qdrant health check failed: {str(e)}")
            return False

    def search(self, query_vector: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in the collection

        Args:
            query_vector: The vector to search for similar items
            top_k: Number of results to return

        Returns:
            List of similar items with payload
        """
        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                with_payload=True,
                with_vectors=False,
                score_threshold=0.3  # Minimum relevance threshold
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    'content': result.payload.get('content', ''),
                    'source_url': result.payload.get('source_url', ''),
                    'chapter': result.payload.get('chapter', ''),
                    'section': result.payload.get('section', ''),
                    'chunk_id': result.id,
                    'relevance_score': result.score,
                    'token_count': result.payload.get('token_count', 0)
                })

            logger.info(f"Search completed, found {len(formatted_results)} results")
            return formatted_results

        except Exception as e:
            logger.error(f"Search failed: {str(e)}")
            return []

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection

        Returns:
            Collection information
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)
            return {
                "name": collection_info.config.params.vectors_count,
                "vectors_count": collection_info.config.params.vectors_count,
                "vector_size": self.vector_size,
                "status": collection_info.status
            }
        except Exception as e:
            logger.error(f"Failed to get collection info: {str(e)}")
            return {"error": str(e)}

    def validate_connection(self) -> bool:
        """
        Validate that the Qdrant connection is working properly

        Returns:
            True if connection is valid, False otherwise
        """
        try:
            # Try to get collection info to verify connection and collection exists
            collection_info = self.client.get_collection(self.collection_name)
            logger.info(f"Connection to collection {self.collection_name} is valid")
            return True
        except Exception as e:
            logger.error(f"Failed to validate Qdrant connection: {str(e)}")
            return False