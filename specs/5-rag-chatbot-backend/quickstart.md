# Quickstart Guide: RAG Chatbot Backend

## Prerequisites

- Python 3.11+
- uv package manager
- Access to OpenAI API
- Access to Cohere API
- Qdrant vector database (cloud instance)

## Setup

### 1. Clone and Navigate to Project

```bash
git clone <repository-url>
cd backend
```

### 2. Install uv Package Manager (if not already installed)

```bash
pip install uv
```

### 3. Set Up Environment

Create a virtual environment and install dependencies:

```bash
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv pip install -e .
```

### 4. Configure Environment Variables

Copy the environment template and add your API keys:

```bash
cp .env.example .env
```

Edit `.env` and add:

```env
OPENAI_API_KEY=your_openai_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_COLLECTION_NAME=your_collection_name
```

## Running the Service

### 1. Start the Backend Service

```bash
cd backend
uv run main.py
```

The service will start on `http://localhost:8000`

### 2. Verify the Service is Running

Check the health endpoint:

```bash
curl http://localhost:8000/api/health
```

## Using the Chat API

### 1. Send a Chat Request

```bash
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key principles of humanoid robotics?",
    "session_id": "session-12345"
  }'
```

### 2. Expected Response

```json
{
  "response": "Humanoid robotics is based on several key principles...",
  "confidence_score": 0.85,
  "citations": [
    {
      "source_url": "https://physical-ai-and-humanoid-robotics-lemon.vercel.app/humanoid-design",
      "chapter": "design-principles",
      "section": "introduction",
      "content_snippet": "Humanoid robots are designed to resemble and mimic human behavior...",
      "relevance_score": 0.92
    }
  ],
  "session_id": "session-12345"
}
```

## Architecture Overview

The RAG Chatbot Backend consists of:

1. **Main Agent Controller** (`main.py`): Orchestrates the entire RAG flow
2. **Retrieval Service**: Retrieves relevant context from Qdrant vector database
3. **Response Generation Service**: Generates answers based on retrieved context
4. **Conversation Manager**: Maintains session state within a user session

## Development

### Running Tests

```bash
uv run pytest
```

### Adding Dependencies

```bash
uv pip add package-name
```

### Updating Dependencies

```bash
uv pip sync requirements.txt
```

## Troubleshooting

### Common Issues

1. **API Keys Not Found**: Ensure your `.env` file is properly configured and loaded
2. **Qdrant Connection Issues**: Verify your Qdrant URL and API key are correct
3. **Response Quality**: Check that your vector database has been properly populated with content

### Debugging

Enable debug logging by setting the environment variable:

```bash
export LOG_LEVEL=DEBUG
```