import os
import logging
from typing import List, Dict, Any, Optional
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http import models

load_dotenv()
logger = logging.getLogger(__name__)

class QdrantService:
    def __init__(self):
        api_key = os.getenv("QDRANT_API_KEY")
        url = os.getenv("QDRANT_URL")
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_humanoid_docs_v3")

        if not api_key or not url:
            raise ValueError("QDRANT_API_KEY and QDRANT_URL must be set")

        self.client = QdrantClient(url=url, api_key=api_key)
        self.vector_size = 1024 
        
        # Optional: Ensure collection exists on startup
        # self._ensure_collection_exists()

    def search(self, query_vector: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Standardized search for modern Qdrant (1.x+) - using query_points method
        """
        try:
            # In Qdrant client version 1.16.2+, the search is done via query_points
            search_result = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=top_k,
                with_payload=True,
                with_vectors=False,
                score_threshold=0.3
            )

            # The result structure may be different in newer versions
            hits = search_result.points if hasattr(search_result, 'points') else search_result

            return [
                {
                    'content': hit.payload.get('content', '') if hasattr(hit, 'payload') and isinstance(hit.payload, dict) else '',
                    'source_url': hit.payload.get('source_url', '') if hasattr(hit, 'payload') and isinstance(hit.payload, dict) else '',
                    'chapter': hit.payload.get('chapter', '') if hasattr(hit, 'payload') and isinstance(hit.payload, dict) else '',
                    'section': hit.payload.get('section', '') if hasattr(hit, 'payload') and isinstance(hit.payload, dict) else '',
                    'chunk_id': str(hit.id),  # Convert to string as expected by model
                    'relevance_score': hit.score,
                    'token_count': hit.payload.get('token_count', 0) if hasattr(hit, 'payload') and isinstance(hit.payload, dict) else 0
                }
                for hit in hits
            ]

        except Exception as e:
            logger.error(f"Search failed: {str(e)}")
            return []

    def get_collection_info(self) -> Dict[str, Any]:
        try:
            info = self.client.get_collection(self.collection_name)
            return {
                "name": self.collection_name,
                "status": info.status,
                "points_count": info.points_count,
                "vector_config": info.config.params.vectors
            }
        except Exception as e:
            logger.error(f"Failed to get info: {str(e)}")
            return {"error": str(e)}