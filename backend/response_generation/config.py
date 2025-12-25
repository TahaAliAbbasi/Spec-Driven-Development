import os
from dotenv import load_dotenv
from typing import Optional

# Load environment variables
load_dotenv()

class Settings:
    """Application settings loaded from environment variables"""

    # OpenRouter API Configuration
    OPENROUTER_API_KEY: str = os.getenv("OPENROUTER_API_KEY", "")
    OPENROUTER_BASE_URL: str = os.getenv("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1")
    OPENROUTER_MODEL: str = os.getenv("OPENROUTER_MODEL", "gpt-4.1-mini")

    # Application Configuration
    APP_ENV: str = os.getenv("APP_ENV", "development")
    LOG_LEVEL: str = os.getenv("LOG_LEVEL", "info")
    HOST: str = os.getenv("HOST", "localhost")
    PORT: int = int(os.getenv("PORT", "8000"))

    # Model Configuration
    MODEL_TEMPERATURE: float = float(os.getenv("MODEL_TEMPERATURE", "0"))
    MAX_TOKENS: int = int(os.getenv("MAX_TOKENS", "2048"))

    # Validation
    @classmethod
    def validate(cls):
        """Validate that required settings are present"""
        # For production, require the API key, but allow mock mode for testing
        # The provider adapter will handle the mock mode functionality
        if cls.MODEL_TEMPERATURE != 0:
            raise ValueError("MODEL_TEMPERATURE must be 0 for deterministic behavior as required by constitutional compliance")

        # Validate OpenRouter API key is provided
        if not cls.OPENROUTER_API_KEY:
            raise ValueError("OPENROUTER_API_KEY environment variable is required")

# Create a settings instance
settings = Settings()

# Validate settings at startup
settings.validate()