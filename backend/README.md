# RAG Chatbot Backend

A comprehensive backend service for a Retrieval-Augmented Generation (RAG) chatbot that answers questions about Physical AI and Humanoid Robotics using a knowledge base stored in Qdrant vector database.

## Table of Contents

- [Features](#features)
- [Architecture](#architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [API Endpoints](#api-endpoints)
- [Testing](#testing)
- [Monitoring](#monitoring)
- [Troubleshooting](#troubleshooting)

## Features

- 🔍 **Retrieval-Augmented Generation**: Combines vector database retrieval with AI response generation
- 🤖 **OpenAI Agent Controller**: Orchestrates the RAG pipeline using OpenAI Assistant API
- 📚 **Knowledge Base**: Stores documents in Qdrant vector database for semantic search
- 🌐 **Real-time Communication**: WebSocket support for real-time chat
- 🛡️ **Security**: API key authentication and input sanitization
- ⚡ **Performance**: Caching and rate limiting for optimal performance
- 📊 **Monitoring**: Built-in performance monitoring and alerting
- 🔧 **Backward Compatibility**: Maintains compatibility with existing services

## Architecture

The backend follows a modular architecture with the following components:

- **Models**: Pydantic data models for request/response validation
- **Agents**: OpenAI Assistant controller for RAG orchestration
- **Services**: Qdrant, Cohere, conversation management, and response services
- **Main**: Unified FastAPI application with all endpoints
- **Monitoring**: Performance monitoring and alerting system

## Prerequisites

- Python 3.9 or higher
- uv package manager
- OpenAI API key
- Cohere API key
- Qdrant Cloud account or self-hosted Qdrant instance

## Installation

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd <repository-name>/backend
   ```

2. **Install uv package manager (if not already installed):**
   ```bash
   # On macOS/Linux:
   curl -LsSf https://astral.sh/uv/install.sh | sh

   # On Windows:
   powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
   ```

3. **Install dependencies:**
   ```bash
   uv sync
   ```

4. **Set up environment variables:**
   Create a `.env` file in the backend directory with the following:
   ```env
   OPENAI_API_KEY=your_openai_api_key
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_COLLECTION_NAME=physical_ai_humanoid_docs_v3
   FRONTEND_API_KEY=your_frontend_api_key
   PORT=8000
   LOG_LEVEL=INFO
   ```

## Configuration

The service can be configured using environment variables:

| Variable | Default | Description |
|----------|---------|-------------|
| `OPENAI_API_KEY` | - | Your OpenAI API key |
| `COHERE_API_KEY` | - | Your Cohere API key |
| `QDRANT_API_KEY` | - | Your Qdrant API key |
| `QDRANT_URL` | - | Your Qdrant URL |
| `QDRANT_COLLECTION_NAME` | `physical_ai_humanoid_docs_v3` | Name of the Qdrant collection |
| `FRONTEND_API_KEY` | - | API key for frontend authentication |
| `PORT` | `8000` | Port to run the service on |
| `LOG_LEVEL` | `INFO` | Logging level (DEBUG, INFO, WARNING, ERROR) |
| `MAX_CONTEXT_TOKENS` | `4000` | Maximum tokens for context |
| `RATE_LIMIT_REQUESTS` | `100` | Rate limit requests per window |
| `RATE_LIMIT_WINDOW` | `60` | Rate limit window in seconds |
| `SESSION_TIMEOUT_MINUTES` | `30` | Session timeout in minutes |
| `DEFAULT_CONFIDENCE_THRESHOLD` | `0.7` | Minimum confidence threshold |

## Usage

### Running the Service

1. **Start the service:**
   ```bash
   cd backend
   uv run python main.py
   ```

2. **Or using uvicorn directly:**
   ```bash
   uv run uvicorn main:app --reload --host 0.0.0.0 --port 8000
   ```

### API Endpoints

The service provides the following endpoints:

#### Primary Chat Endpoint
- `POST /api/chat` - Main chat endpoint for RAG-powered responses

#### Backward Compatible Endpoints
- `POST /api/retrieve` - Retrieve relevant context from knowledge base
- `POST /api/answer` - Generate response from query and context

#### Utility Endpoints
- `GET /api/health` - Health check endpoint
- `GET /` - Root endpoint
- `WebSocket /ws/chat` - Real-time chat communication

### Environment Setup for Development

1. **Create a virtual environment:**
   ```bash
   uv venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   ```

2. **Install in development mode:**
   ```bash
   uv sync --dev
   ```

## Testing

### Running Tests

1. **Unit Tests:**
   ```bash
   uv run pytest tests/unit/
   ```

2. **Integration Tests:**
   ```bash
   uv run pytest tests/integration/
   ```

3. **All Tests:**
   ```bash
   uv run pytest tests/
   ```

### Test Coverage

- Core component functionality
- RAG pipeline integration
- API endpoint behavior
- Error handling scenarios
- Performance benchmarks

## Monitoring

The service includes built-in monitoring capabilities:

- **Metrics Collection**: Request counts, response times, error rates
- **Performance Monitoring**: Response time thresholds, error rate thresholds
- **Alerting**: Automatic alerts for performance and error issues
- **Health Checks**: Service availability and dependency health

### Accessing Metrics

Metrics are available through the health endpoint and can be integrated with external monitoring systems.

## Troubleshooting

### Common Issues

#### Service Won't Start
- Check that all required environment variables are set
- Verify API keys are valid and have proper permissions
- Ensure Qdrant is accessible

#### Slow Responses
- Check Qdrant connection and performance
- Verify Cohere and OpenAI API availability
- Monitor system resources

#### Authentication Errors
- Verify API keys are correct
- Check that the API key is being sent in the Authorization header
- Ensure the FRONTEND_API_KEY matches what the frontend is using

### Performance Tips

- Use sessions to maintain conversation context
- Leverage the caching system for repeated queries
- Monitor response times to identify performance bottlenecks
- Use WebSocket for real-time applications to reduce latency

### Debugging

Enable DEBUG logging for more detailed information:
```env
LOG_LEVEL=DEBUG
```

## Deployment

### Production Deployment

1. **Build the application:**
   ```bash
   uv sync --locked
   ```

2. **Run in production:**
   ```bash
   uv run uvicorn main:app --host 0.0.0.0 --port $PORT
   ```

### Docker Deployment (Coming Soon)

Dockerfile will be available for containerized deployment.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

### Development Guidelines

- Follow PEP 8 coding standards
- Write comprehensive tests for new features
- Update documentation for API changes
- Use type hints for all function parameters and return values

## Support

For support, please create an issue in the repository with:
- Detailed description of the problem
- Steps to reproduce
- Expected vs. actual behavior
- Environment information

## License

[Specify your license here]

## Acknowledgments

- OpenAI for the Assistant API
- Cohere for embedding services
- Qdrant for vector database technology
- FastAPI for the web framework