# Phase 3 - Agent-Based Response Generation Service

This service implements the response generation component of the RAG system, using OpenAI Agents SDK as an orchestration layer with OpenRouter as the underlying language model provider for GPT-class models.

## Architecture

The system uses a provider adapter pattern to enable compatibility between the OpenAI Agents SDK and the OpenRouter API:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   User Query    │───▶│  RAGAnswerAgent │───▶│   Response      │
│   & Context     │    │                 │    │   with Citations│
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │
                       ┌─────────────────┐
                       │ Provider Adapter│
                       │ (OpenAI SDK -> │
                       │  OpenRouter API) │
                       └─────────────────┘
                              │
                       ┌─────────────────┐
                       │  OpenRouter API │
                       │ (GPT models)    │
                       └─────────────────┘
```

## Features

- Constitutional compliance with zero hallucination policy
- Deterministic responses with temperature=0
- Source attribution with citations
- Selected-text-only mode enforcement
- "NO RETRIEVAL" hard guard
- Fast dependency management with uv package manager

## Setup

1. Install uv package manager (if not already installed):
   ```bash
   pip install uv
   # Or follow installation instructions at https://github.com/astral-sh/uv
   ```

2. Install dependencies with uv:
   ```bash
   uv sync
   # Or for development: uv sync --dev
   ```

3. Set up environment variables:
   ```bash
   cp .env .env.local  # then edit with your actual API keys
   ```

4. Run the service:
   ```bash
   uv run python main.py
   # Or with uvicorn: uv run uvicorn main:app --reload
   ```

## API Endpoints

- `POST /api/answer` - Generate response from query and context bundle
- `GET /api/health/answer` - Health check for the service

## Configuration

- `OPENROUTER_API_KEY`: Your OpenRouter API key
- `OPENROUTER_BASE_URL`: OpenRouter API base URL (default: https://openrouter.ai/api/v1)
- `OPENROUTER_MODEL`: Model identifier (default: gpt-4.1-mini)
- `MODEL_TEMPERATURE`: Must be 0 for deterministic behavior (enforced)
- `MAX_TOKENS`: Maximum tokens for response generation
- `LOG_LEVEL`: Logging level (info, debug, warning, error)

## Error Handling

- 400: Invalid input format
- 422: Constitutional violation (insufficient context, selected-text-only violation, etc.)
- 500: Agent processing error