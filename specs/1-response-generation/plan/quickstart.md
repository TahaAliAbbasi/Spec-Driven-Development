# Quickstart Guide: Phase 3 - Agent-Based Response Generation

**Feature**: Phase 3 - Agent-Based Response Generation & Orchestration
**Created**: 2025-12-13

## Overview

This guide provides instructions for setting up and running the agent-based response generation service. The service consumes ContextBundle objects from Phase 2 and generates constitutionally-compliant responses using OpenAI Agents SDK.

## Prerequisites

- Python 3.9+
- OpenAI API key
- Access to Phase 2 ContextBundle objects
- FastAPI-compatible environment

## Setup

1. **Install Dependencies**
   ```bash
   pip install fastapi openai pydantic python-dotenv uvicorn
   ```

2. **Environment Configuration**
   Create a `.env` file with the following variables:
   ```
   OPENAI_API_KEY=your_openai_api_key_here
   PHASE_2_API_URL=http://localhost:8001  # URL to Phase 2 service
   TEMPERATURE=0  # For deterministic responses
   ```

3. **Run the Service**
   ```bash
   uvicorn main:app --reload --port 8000
   ```

## Usage

### API Endpoint
- POST `/api/answer` - Generate response from query and context bundle

### Example Request
```json
{
  "query": "What are the principles of RAG systems?",
  "context_bundle": {
    "chunks": [
      {
        "chunk_id": "chunk_123",
        "content": "RAG systems combine retrieval and generation...",
        "metadata": {"source": "document.pdf", "page": 5}
      }
    ],
    "status": "success"
  },
  "mode": "global"
}
```

### Example Response
```json
{
  "answer": "RAG systems combine retrieval and generation to provide contextually relevant responses...",
  "citations": [
    {
      "chunk_id": "chunk_123",
      "source_url": "document.pdf"
    }
  ],
  "used_chunks": ["chunk_123"],
  "status": "answered",
  "warnings": []
}
```

## Configuration Options

- `TEMPERATURE`: Set to 0 for deterministic responses (required for constitutional compliance)
- `MAX_TOKENS`: Maximum tokens for response generation
- `CONTEXT_TRUNCATION_LIMIT`: Maximum context size before truncation (though truncation is prohibited at Phase 3 level)

## Testing

Run the service tests:
```bash
python -m pytest tests/
```

## Troubleshooting

- **500 errors**: Check OpenAI API key validity and rate limits
- **422 errors**: Verify ContextBundle format matches Phase 2 output
- **Hallucination issues**: Ensure temperature is set to 0 and constitutional checks are active