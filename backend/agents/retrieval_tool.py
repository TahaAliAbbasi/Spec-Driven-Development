"""
Agent tool for context retrieval
"""
import os
import logging
from typing import Dict, Any, List
from dotenv import load_dotenv

from services.qdrant_service import QdrantService
from services.cohere_service import CohereService
from models.context import RetrievedContext

# Load environment variables
load_dotenv()

# Configure logging
logger = logging.getLogger(__name__)


class RetrievalTool:
    """
    Agent tool for retrieving context from the knowledge base
    """

    def __init__(self):
        """
        Initialize the retrieval tool with required services
        """
        self.qdrant_service = QdrantService()
        self.cohere_service = CohereService()
        logger.info("RetrievalTool initialized")

    def retrieve_context(self, query: str, top_k: int = 5) -> List[RetrievedContext]:
        """
        Retrieve context based on a query

        Args:
            query: The query to retrieve context for
            top_k: Number of results to retrieve

        Returns:
            List of RetrievedContext objects
        """
        try:
            # Generate embedding for the query
            query_embedding = self.cohere_service.embed_text(query)

            # Search in Qdrant for relevant context
            search_results = self.qdrant_service.search(query_embedding, top_k=top_k)

            # Convert search results to RetrievedContext objects
            retrieved_context = []
            for result in search_results:
                context = RetrievedContext(
                    content=result['content'],
                    source_url=result['source_url'],
                    chapter=result['chapter'],
                    section=result['section'],
                    chunk_id=result['chunk_id'],
                    relevance_score=result['relevance_score'],
                    token_count=result['token_count']
                )
                retrieved_context.append(context)

            logger.info(f"Retrieved {len(retrieved_context)} context chunks for query")
            return retrieved_context

        except Exception as e:
            logger.error(f"Error retrieving context: {str(e)}")
            return []

    def validate_retrieval(self, context_list: List[RetrievedContext], min_relevance: float = 0.3) -> bool:
        """
        Validate that the retrieved context meets quality requirements

        Args:
            context_list: List of retrieved context chunks
            min_relevance: Minimum relevance score for valid context

        Returns:
            True if context meets quality requirements, False otherwise
        """
        if not context_list:
            return False

        # Check if any context has sufficient relevance
        for context in context_list:
            if context.relevance_score >= min_relevance:
                return True

        return False

    def get_retrieval_stats(self) -> Dict[str, Any]:
        """
        Get statistics about the retrieval service

        Returns:
            Dictionary with retrieval service statistics
        """
        return {
            "qdrant_connection": self.qdrant_service.health_check(),
            "cohere_connection": self.cohere_service.health_check(),
            "vector_size": self.cohere_service.get_model_info()["vector_size"]
        }