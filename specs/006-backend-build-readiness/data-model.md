# Data Model: Backend Build Readiness

## Key Entities

### Build Configuration
- **Fields**: dependencies, environment variables, build scripts, deployment settings
- **Validation**: All required dependencies must be specified in pyproject.toml and uv.lock
- **Relationships**: Used by deployment process to build and run the application

### Environment Variables
- **Fields**: API keys (OPENAI_API_KEY, COHERE_API_KEY, QDRANT_API_KEY), configuration settings (PORT, LOG_LEVEL, etc.)
- **Validation**: All sensitive values must be loaded from environment variables, not hardcoded
- **Relationships**: Used by application at runtime for configuration

### Deployment Artifacts
- **Fields**: Dockerfile, docker-compose.yml, deployment scripts, configuration files
- **Validation**: Must not contain hardcoded credentials or sensitive information
- **Relationships**: Used to package and deploy the application

### Runtime Dependencies
- **Fields**: External services (Qdrant, OpenAI, Cohere), system libraries
- **Validation**: Must be available in deployment environment
- **Relationships**: Required for application to function properly

## Configuration Validation

### Required Environment Variables
- OPENAI_API_KEY: Required for OpenAI API access
- COHERE_API_KEY: Required for Cohere API access
- QDRANT_API_KEY: Required for Qdrant database access
- QDRANT_URL: Required for Qdrant database connection
- QDRANT_COLLECTION_NAME: Required for Qdrant collection access

### Optional Environment Variables
- PORT: Port to run the service on (default: 8000)
- LOG_LEVEL: Logging level (default: INFO)
- MAX_CONTEXT_TOKENS: Maximum tokens for context (default: 4000)
- RATE_LIMIT_REQUESTS: Rate limit requests per window (default: 100)
- RATE_LIMIT_WINDOW: Rate limit window in seconds (default: 60)
- ALLOWED_ORIGINS: Comma-separated list of allowed origins for CORS (default: none)