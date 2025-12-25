# Retrieval & Context Assembly Service

This service implements Phase 2 of the AI-driven book project with RAG chatbot. It accepts user queries or selected text, converts them to vector representations, and retrieves the most relevant content chunks from the Qdrant vector database based on cosine similarity.

## Features

- **Query-based retrieval**: Search for relevant content using natural language queries
- **Selected-text retrieval**: Focus on content semantically related to selected text
- **Metadata filtering**: Filter results by chapter, section, language, and version
- **Context assembly**: Bundle relevant chunks within token limits
- **Zero hallucination**: Only returns content present in the vector database

## API Endpoints

### POST /api/retrieve
Retrieve relevant context based on query or selected text

**Request Body:**
```json
{
  "query": "What are the key concepts in humanoid robotics?",
  "selected_text": null,
  "top_k": 5,
  "chapter_filter": null,
  "section_filter": null,
  "language_constraint": "en",
  "version_constraint": "1.0"
}
```

**Response:**
```json
{
  "retrieved_chunks": [
    {
      "content": "Humanoid robots are robots with physical characteristics resembling humans...",
      "source_url": "https://physical-ai-and-humanoid-robotics-lemon.vercel.app/introduction",
      "chapter": "introduction",
      "section": "overview",
      "chunk_id": "123e4567-e89b-12d3-a456-426614174000",
      "relevance_score": 0.85,
      "token_count": 45
    }
  ],
  "metadata": {
    "query_type": "standard",
    "processing_time_ms": 125,
    "original_top_k": 5,
    "returned_count": 3
  },
  "status": "success",
  "total_tokens": 135
}
```

### GET /api/health/retrieval
Health check for retrieval service

**Response:**
```json
{
  "status": "healthy",
  "timestamp": 1702568400.123456,
  "service": "retrieval",
  "dependencies": {
    "qdrant": "connected",
    "cohere": "available"
  }
}
```

## Environment Variables

- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_URL`: Your Qdrant Cloud endpoint URL
- `QDRANT_API_KEY`: Your Qdrant Cloud API key
- `MAX_CONTEXT_TOKENS`: Maximum tokens in returned context (default: 4000)
- `LOG_LEVEL`: Logging level (default: INFO)
- `PORT`: Port to run the service on (default: 8000)
- `QDRANT_COLLECTION_NAME`: Name of the collection in Qdrant (default: physical_ai_humanoid_docs_v3)

## Running the Service

1. Install dependencies: `uv sync`
2. Set up environment variables in a `.env` file
3. Run the service: `uv run retrieval/main.py`

## Configuration

The service is configured through environment variables. Make sure to set up your Cohere and Qdrant credentials before running the service.

## Architecture

The service implements a single-file architecture with the following components:
- Data models using Pydantic
- Vectorization service using Cohere
- Similarity search engine using Qdrant
- Metadata filtering layer
- Context assembly engine
- FastAPI endpoints