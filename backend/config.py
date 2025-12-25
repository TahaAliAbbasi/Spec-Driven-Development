"""
Unified configuration management for the RAG Chatbot Backend
"""
import os
from typing import Optional
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class Config:
    """
    Unified configuration class for all application settings
    """

    # API Keys
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "")
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "")
    QDRANT_URL: str = os.getenv("QDRANT_URL", "")

    # Qdrant Configuration
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_humanoid_docs_v3")

    # Application Settings
    LOG_LEVEL: str = os.getenv("LOG_LEVEL", "INFO")
    PORT: int = int(os.getenv("PORT", "8000"))
    MAX_CONTEXT_TOKENS: int = int(os.getenv("MAX_CONTEXT_TOKENS", "4000"))
    CONTENT_VERSION: str = os.getenv("CONTENT_VERSION", "1.0")

    # Rate Limiting
    RATE_LIMIT_REQUESTS: int = int(os.getenv("RATE_LIMIT_REQUESTS", "100"))
    RATE_LIMIT_WINDOW: int = int(os.getenv("RATE_LIMIT_WINDOW", "60"))  # in seconds

    # Session Configuration
    SESSION_TIMEOUT_MINUTES: int = int(os.getenv("SESSION_TIMEOUT_MINUTES", "30"))

    # Confidence Settings
    DEFAULT_CONFIDENCE_THRESHOLD: float = float(os.getenv("DEFAULT_CONFIDENCE_THRESHOLD", "0.7"))
    LOW_CONFIDENCE_MESSAGE: str = os.getenv(
        "LOW_CONFIDENCE_MESSAGE",
        "I couldn't find sufficient information in the knowledge base to answer your question confidently. Please try rephrasing your query."
    )

    # Service URLs
    RETRIEVAL_SERVICE_URL: str = os.getenv("RETRIEVAL_SERVICE_URL", "http://localhost:8000")
    RESPONSE_SERVICE_URL: str = os.getenv("RESPONSE_SERVICE_URL", "http://localhost:8001")
    SERVICE_TIMEOUT: int = int(os.getenv("SERVICE_TIMEOUT", "30"))

    # Model Configuration
    OPENAI_MODEL: str = os.getenv("OPENAI_MODEL", "gpt-4-turbo")
    COHERE_MODEL: str = os.getenv("COHERE_MODEL", "embed-multilingual-v3.0")

    @classmethod
    def validate_config(cls) -> bool:
        """
        Validate that required configuration values are set

        Returns:
            True if configuration is valid, False otherwise
        """
        required_keys = [
            'OPENAI_API_KEY',
            'COHERE_API_KEY',
            'QDRANT_API_KEY',
            'QDRANT_URL'
        ]

        for key in required_keys:
            value = getattr(cls, key)
            if not value:
                print(f"Configuration error: {key} is not set")
                return False

        return True

    @classmethod
    def get_database_config(cls) -> dict:
        """
        Get database-specific configuration

        Returns:
            Dictionary with database configuration
        """
        return {
            "qdrant_url": cls.QDRANT_URL,
            "qdrant_api_key": cls.QDRANT_API_KEY,
            "collection_name": cls.QDRANT_COLLECTION_NAME
        }

    @classmethod
    def get_service_config(cls) -> dict:
        """
        Get service-specific configuration

        Returns:
            Dictionary with service configuration
        """
        return {
            "retrieval_service_url": cls.RETRIEVAL_SERVICE_URL,
            "response_service_url": cls.RESPONSE_SERVICE_URL,
            "service_timeout": cls.SERVICE_TIMEOUT
        }

    @classmethod
    def get_ai_config(cls) -> dict:
        """
        Get AI/ML-specific configuration

        Returns:
            Dictionary with AI configuration
        """
        return {
            "openai_model": cls.OPENAI_MODEL,
            "cohere_model": cls.COHERE_MODEL,
            "default_confidence_threshold": cls.DEFAULT_CONFIDENCE_THRESHOLD
        }