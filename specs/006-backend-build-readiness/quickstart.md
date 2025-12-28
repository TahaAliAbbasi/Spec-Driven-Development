# Quickstart: Backend Build Readiness

## Prerequisites

- Python 3.11+
- Docker and Docker Compose
- Access to API keys (OpenAI, Cohere, Qdrant)

## Local Development Setup

1. **Clone and navigate to backend directory:**
   ```bash
   cd backend
   ```

2. **Install dependencies using uv:**
   ```bash
   uv sync
   # Or if uv is not available:
   pip install -r requirements.txt
   ```

3. **Set up environment variables:**
   ```bash
   cp .env.example .env
   # Edit .env with your actual API keys
   ```

4. **Run the application:**
   ```bash
   uv run uvicorn main:app --reload --port 8000
   # Or if uv is not available:
   python -m uvicorn main:app --reload --port 8000
   ```

## Containerized Deployment

1. **Prepare for deployment:**
   ```bash
   ./scripts/prepare-deployment.sh
   ```

2. **Build and run with Docker Compose:**
   ```bash
   cd deployment
   docker-compose up --build
   ```

3. **Or build and run the Docker image directly:**
   ```bash
   # From backend/deployment directory
   docker build -t rag-chatbot-backend -f deployment/Dockerfile ../
   docker run -d --name rag-chatbot-backend -p 8000:8000 rag-chatbot-backend
   ```

## Testing

1. **Run tests with the test runner:**
   ```bash
   python scripts/run_tests.py
   # Or directly:
   PYTHONPATH=. python -m pytest tests/ -v
   ```

## Health Check

Verify the application is running:
- API Health: `GET http://localhost:8000/api/health`
- Root endpoint: `GET http://localhost:8000/`

## Configuration

The application can be configured using environment variables:

| Variable | Description | Default |
|----------|-------------|---------|
| PORT | Port to run the service | 8000 |
| LOG_LEVEL | Logging level | INFO |
| ALLOWED_ORIGINS | Comma-separated list of allowed origins | (empty) |
| MAX_CONTEXT_TOKENS | Maximum tokens for context | 4000 |