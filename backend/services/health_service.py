"""
Service status tracking and health check service
"""
import logging
import time
from typing import Dict, Any, List
from datetime import datetime
from enum import Enum

from ..services.qdrant_service import QdrantService
from ..services.cohere_service import CohereService
from ..agents.chat_agent import ChatAgent

# Configure logging
logger = logging.getLogger(__name__)


class ServiceStatus(Enum):
    """
    Enum for service status
    """
    HEALTHY = "healthy"
    UNSTABLE = "unstable"
    UNAVAILABLE = "unavailable"
    DEGRADED = "degraded"


class HealthService:
    """
    Service for tracking and reporting service health status
    """

    def __init__(self):
        """
        Initialize the health service
        """
        self.last_health_check = None
        self.service_status = ServiceStatus.HEALTHY
        self.check_results = {}
        self.last_check_time = None
        logger.info("HealthService initialized")

    def perform_health_check(self) -> Dict[str, Any]:
        """
        Perform a comprehensive health check of all services

        Returns:
            Dictionary with health check results
        """
        start_time = time.time()
        self.last_check_time = datetime.now()

        # Initialize services for health check
        qdrant_service = QdrantService()
        cohere_service = CohereService()
        chat_agent = ChatAgent()

        # Check each service
        self.check_results = {
            "qdrant": self._check_qdrant_service(qdrant_service),
            "cohere": self._check_cohere_service(cohere_service),
            "openai": self._check_openai_service(chat_agent),
            "app": self._check_app_health()
        }

        # Determine overall status
        status_scores = {
            ServiceStatus.HEALTHY: 0,
            ServiceStatus.DEGRADED: 1,
            ServiceStatus.UNSTABLE: 2,
            ServiceStatus.UNAVAILABLE: 3
        }

        max_status = ServiceStatus.HEALTHY
        for service_result in self.check_results.values():
            service_status = ServiceStatus(service_result["status"])
            if status_scores[service_status] > status_scores[max_status]:
                max_status = service_status

        self.service_status = max_status
        self.last_health_check = datetime.now()

        health_result = {
            "status": self.service_status.value,
            "timestamp": self.last_health_check.isoformat(),
            "check_duration": time.time() - start_time,
            "services": self.check_results,
            "overall_health": self.service_status.value
        }

        logger.info(f"Health check completed with status: {self.service_status.value}")
        return health_result

    def _check_qdrant_service(self, qdrant_service: QdrantService) -> Dict[str, Any]:
        """
        Check the health of the Qdrant service

        Args:
            qdrant_service: QdrantService instance to check

        Returns:
            Health check result for Qdrant
        """
        try:
            is_healthy = qdrant_service.health_check()
            status = ServiceStatus.HEALTHY if is_healthy else ServiceStatus.UNAVAILABLE
            return {
                "status": status.value,
                "available": is_healthy,
                "message": "Qdrant service is accessible" if is_healthy else "Qdrant service is unavailable",
                "timestamp": datetime.now().isoformat()
            }
        except Exception as e:
            logger.error(f"Qdrant health check failed: {str(e)}")
            return {
                "status": ServiceStatus.UNAVAILABLE.value,
                "available": False,
                "message": f"Qdrant service error: {str(e)}",
                "timestamp": datetime.now().isoformat()
            }

    def _check_cohere_service(self, cohere_service: CohereService) -> Dict[str, Any]:
        """
        Check the health of the Cohere service

        Args:
            cohere_service: CohereService instance to check

        Returns:
            Health check result for Cohere
        """
        try:
            is_healthy = cohere_service.health_check()
            status = ServiceStatus.HEALTHY if is_healthy else ServiceStatus.UNAVAILABLE
            return {
                "status": status.value,
                "available": is_healthy,
                "message": "Cohere service is accessible" if is_healthy else "Cohere service is unavailable",
                "timestamp": datetime.now().isoformat()
            }
        except Exception as e:
            logger.error(f"Cohere health check failed: {str(e)}")
            return {
                "status": ServiceStatus.UNAVAILABLE.value,
                "available": False,
                "message": f"Cohere service error: {str(e)}",
                "timestamp": datetime.now().isoformat()
            }

    def _check_openai_service(self, chat_agent: ChatAgent) -> Dict[str, Any]:
        """
        Check the health of the OpenAI service through the chat agent

        Args:
            chat_agent: ChatAgent instance to check

        Returns:
            Health check result for OpenAI
        """
        try:
            # Test with a simple operation
            test_result = chat_agent.validate_query("test")
            is_healthy = test_result is not None
            status = ServiceStatus.HEALTHY if is_healthy else ServiceStatus.UNSTABLE
            return {
                "status": status.value,
                "available": is_healthy,
                "message": "OpenAI service is accessible" if is_healthy else "OpenAI service validation failed",
                "timestamp": datetime.now().isoformat()
            }
        except Exception as e:
            logger.error(f"OpenAI health check failed: {str(e)}")
            return {
                "status": ServiceStatus.UNAVAILABLE.value,
                "available": False,
                "message": f"OpenAI service error: {str(e)}",
                "timestamp": datetime.now().isoformat()
            }

    def _check_app_health(self) -> Dict[str, Any]:
        """
        Check the health of the application itself

        Returns:
            Health check result for the app
        """
        return {
            "status": ServiceStatus.HEALTHY.value,
            "available": True,
            "message": "Application is running normally",
            "timestamp": datetime.now().isoformat()
        }

    def get_current_status(self) -> Dict[str, Any]:
        """
        Get the current service status without performing a full health check

        Returns:
            Current status information
        """
        return {
            "status": self.service_status.value,
            "last_check": self.last_check_time.isoformat() if self.last_check_time else None,
            "timestamp": datetime.now().isoformat()
        }

    def is_service_healthy(self) -> bool:
        """
        Check if the service is considered healthy

        Returns:
            True if service is healthy, False otherwise
        """
        return self.service_status == ServiceStatus.HEALTHY

    def get_degraded_services(self) -> List[str]:
        """
        Get a list of services that are not healthy

        Returns:
            List of service names that are not healthy
        """
        degraded = []
        for service_name, result in self.check_results.items():
            if result["status"] != ServiceStatus.HEALTHY.value:
                degraded.append(service_name)
        return degraded


# Global health service instance
health_service = HealthService()