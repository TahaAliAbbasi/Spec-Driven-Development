# Quickstart Guide: Retrieval & Context Assembly

## Prerequisites

- Python 3.9+
- uv package manager
- Cohere API key
- Qdrant Cloud access with populated vector database from Phase 1

## Setup

1. **Install dependencies**:
   ```bash
   cd backend
   uv sync
   ```

2. **Configure environment variables**:
   ```bash
   # Create .env file with:
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cloud_url
   QDRANT_API_KEY=your_qdrant_api_key
   MAX_CONTEXT_TOKENS=4000
   ```

3. **Verify Phase 1 data**:
   Ensure the vector database contains content from Phase 1 before starting retrieval service.

## Running the Service

1. **Start the service**:
   ```bash
   cd backend
   uv run retrieval/main.py
   ```

2. **Test the API**:
   ```bash
   curl -X POST http://localhost:8000/api/retrieve \
     -H "Content-Type: application/json" \
     -d '{
       "query": "What are the key concepts in humanoid robotics?",
       "top_k": 5
     }'
   ```

## Basic Usage

### Standard Query Retrieval
```python
{
  "query": "Explain neural networks in robotics",
  "top_k": 5,
  "language_constraint": "en"
}
```

### Selected-Text Retrieval
```python
{
  "selected_text": "The robot uses sensors to perceive its environment",
  "top_k": 3,
  "chapter_filter": "sensors"
}
```

## Configuration Options

- `MAX_CONTEXT_TOKENS`: Maximum tokens in returned context (default: 4000)
- `DEFAULT_TOP_K`: Default number of results (default: 5)
- `MIN_RELEVANCE_SCORE`: Minimum similarity threshold (default: 0.3)

## Health Check

Verify service is running:
```bash
curl http://localhost:8000/api/health/retrieval
```