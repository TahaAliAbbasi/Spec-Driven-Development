"""
Service integration layer for connecting to existing services
"""
import os
import logging
from typing import Dict, Any, Optional
import httpx
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logger = logging.getLogger(__name__)


class ServiceIntegration:
    """
    Service integration layer for connecting to existing retrieval and response generation services
    """

    def __init__(self):
        """
        Initialize the service integration layer
        """
        self.retrieval_service_url = os.getenv("RETRIEVAL_SERVICE_URL", "http://localhost:8000")
        self.response_service_url = os.getenv("RESPONSE_SERVICE_URL", "http://localhost:8001")
        self.timeout = int(os.getenv("SERVICE_TIMEOUT", "30"))
        logger.info("ServiceIntegration initialized")

    async def call_retrieval_service(self, query: str, top_k: int = 5) -> Dict[str, Any]:
        """
        Call the existing retrieval service to get relevant context

        Args:
            query: The user query to retrieve context for
            top_k: Number of results to retrieve

        Returns:
            Response from the retrieval service
        """
        try:
            async with httpx.AsyncClient(timeout=self.timeout) as client:
                response = await client.post(
                    f"{self.retrieval_service_url}/api/retrieve",
                    json={
                        "query": query,
                        "top_k": top_k
                    },
                    headers={"Content-Type": "application/json"}
                )
                response.raise_for_status()
                result = response.json()
                logger.info(f"Retrieved context from service: {len(result.get('retrieved_chunks', []))} chunks")
                return result
        except httpx.RequestError as e:
            logger.error(f"Request error calling retrieval service: {str(e)}")
            return {"status": "error", "message": f"Retrieval service unavailable: {str(e)}"}
        except httpx.HTTPStatusError as e:
            logger.error(f"HTTP error calling retrieval service: {str(e)}")
            return {"status": "error", "message": f"Retrieval service returned error: {str(e)}"}

    async def call_response_generation_service(self, query: str, context: Dict[str, Any]) -> Dict[str, Any]:
        """
        Call the existing response generation service to generate response

        Args:
            query: The user query
            context: Retrieved context to base the response on

        Returns:
            Response from the response generation service
        """
        try:
            async with httpx.AsyncClient(timeout=self.timeout) as client:
                response = await client.post(
                    f"{self.response_service_url}/api/answer",
                    json={
                        "query": query,
                        "context_bundle": context
                    },
                    headers={"Content-Type": "application/json"}
                )
                response.raise_for_status()
                result = response.json()
                logger.info("Generated response from service")
                return result
        except httpx.RequestError as e:
            logger.error(f"Request error calling response generation service: {str(e)}")
            return {"status": "error", "message": f"Response generation service unavailable: {str(e)}"}
        except httpx.HTTPStatusError as e:
            logger.error(f"HTTP error calling response generation service: {str(e)}")
            return {"status": "error", "message": f"Response generation service returned error: {str(e)}"}

    def validate_service_urls(self) -> bool:
        """
        Validate that the service URLs are properly configured

        Returns:
            True if URLs are valid, False otherwise
        """
        if not self.retrieval_service_url or not self.response_service_url:
            logger.error("Service URLs not properly configured")
            return False

        # Basic validation - check if they look like URLs
        return (
            self.retrieval_service_url.startswith(("http://", "https://")) and
            self.response_service_url.startswith(("http://", "https://"))
        )