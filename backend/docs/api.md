# RAG Chatbot Backend API Documentation

## Overview

The RAG Chatbot Backend provides a comprehensive API for interacting with a Retrieval-Augmented Generation (RAG) system that answers questions about Physical AI and Humanoid Robotics using a knowledge base stored in Qdrant vector database.

## Base URL

```
http://localhost:8000
```

## Authentication

All API endpoints require authentication using an API key in the Authorization header:

```
Authorization: Bearer YOUR_API_KEY
```

## Rate Limiting

All endpoints are subject to rate limiting:
- Chat endpoint: 100 requests per minute
- Retrieve endpoint: 75 requests per minute
- Answer endpoint: 60 requests per minute
- Health endpoint: 200 requests per minute

## Endpoints

### 1. Chat Endpoint (Primary)

#### POST `/api/chat`

Sends a user query and receives an AI-generated response based on the knowledge base with citations.

**Headers:**
- `Authorization: Bearer YOUR_API_KEY`
- `Content-Type: application/json`

**Request Body:**
```json
{
  "query_text": "Your question about Physical AI and Humanoid Robotics",
  "session_id": "optional session ID for multi-turn conversations",
  "timestamp": "2023-12-24T10:00:00.000Z",
  "user_id": "optional user identifier"
}
```

**Request Fields:**
- `query_text` (string, required): The question or query text (1-1000 characters)
- `session_id` (string, optional): Session identifier for maintaining conversation context
- `timestamp` (string, optional): Timestamp of the query (defaults to current time)
- `user_id` (string, optional): User identifier for tracking purposes

**Response:**
```json
{
  "response_text": "The AI-generated response to your query",
  "citations": [
    {
      "title": "Title of the source",
      "url": "https://example.com/source",
      "author": "Author Name",
      "relevance_score": 0.85
    }
  ],
  "confidence_score": 0.92,
  "session_id": "session identifier",
  "timestamp": "2023-12-24T10:00:00.000Z"
}
```

**Response Fields:**
- `response_text` (string): The AI-generated response
- `citations` (array): List of sources used to generate the response
- `confidence_score` (number): Confidence level of the response (0.0-1.0)
- `session_id` (string): Session identifier
- `timestamp` (string): When the response was generated

### 2. Retrieve Endpoint (Backward Compatible)

#### POST `/api/retrieve`

Retrieves relevant context from the knowledge base without generating a response.

**Headers:**
- `Authorization: Bearer YOUR_API_KEY`
- `Content-Type: application/json`

**Request Body:**
```json
{
  "query": "Search query for relevant documents",
  "selected_text": "optional selected text",
  "top_k": 5,
  "chapter_filter": "optional chapter filter",
  "section_filter": "optional section filter",
  "language_constraint": "en",
  "version_constraint": "1.0"
}
```

**Request Fields:**
- `query` (string, optional): Search query for relevant documents
- `selected_text` (string, optional): Selected text to search around
- `top_k` (integer, default: 5): Number of results to return
- `chapter_filter` (string, optional): Filter by specific chapter
- `section_filter` (string, optional): Filter by specific section
- `language_constraint` (string, default: "en"): Language constraint
- `version_constraint` (string, default: "1.0"): Version constraint

**Response:**
```json
{
  "retrieved_chunks": [
    {
      "content": "Content of the retrieved chunk",
      "source_url": "https://example.com/source",
      "chapter": "Chapter name",
      "section": "Section name",
      "chunk_id": "unique chunk identifier",
      "relevance_score": 0.85,
      "token_count": 120
    }
  ]
}
```

### 3. Answer Endpoint (Backward Compatible)

#### POST `/api/answer`

Generates a response based on a query and optional context bundle.

**Headers:**
- `Authorization: Bearer YOUR_API_KEY`
- `Content-Type: application/json`

**Request Body:**
```json
{
  "query": "Your question",
  "context_bundle": {
    "retrieved_chunks": [
      {
        "content": "Context content",
        "source_url": "https://example.com/source",
        "chapter": "Chapter name",
        "section": "Section name",
        "chunk_id": "chunk identifier",
        "relevance_score": 0.85,
        "token_count": 120
      }
    ]
  },
  "mode": "standard"
}
```

**Request Fields:**
- `query` (string, required): The question to answer
- `context_bundle` (object, optional): Pre-retrieved context
- `mode` (string, default: "standard"): Processing mode

**Response:**
```json
{
  "response": "The AI-generated response",
  "status": "success",
  "citations": [
    {
      "title": "Source Title",
      "url": "https://example.com/source",
      "author": "Author Name",
      "relevance_score": 0.85
    }
  ],
  "confidence_score": 0.92
}
```

### 4. Health Check Endpoint

#### GET `/api/health`

Checks the health status of the service.

**Response:**
```json
{
  "status": "healthy",
  "service": "rag-chatbot-backend",
  "timestamp": "2023-12-24T10:00:00.000Z"
}
```

### 5. WebSocket Endpoint (Real-time)

#### WebSocket `/ws/chat`

Provides real-time bidirectional communication for chat functionality.

**Message Format:**
```json
{
  "query": "Your question",
  "session_id": "optional session ID"
}
```

**Response Format:**
```json
{
  "type": "response|typing|error",
  "response": "AI-generated response (for type=response)",
  "message": "Status message (for type=typing|error)",
  "citations": [...],
  "confidence_score": 0.92,
  "session_id": "session ID",
  "timestamp": "2023-12-24T10:00:00.000Z"
}
```

## Error Responses

All endpoints return standard error responses when issues occur:

```json
{
  "detail": "Error message describing the issue"
}
```

Common HTTP status codes:
- `200`: Success
- `400`: Bad Request (invalid input)
- `401`: Unauthorized (invalid API key)
- `429`: Too Many Requests (rate limit exceeded)
- `500`: Internal Server Error

## Configuration

The service can be configured using environment variables in the `.env` file:

```env
OPENAI_API_KEY=your_openai_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_URL=your_qdrant_url
QDRANT_COLLECTION_NAME=physical_ai_humanoid_docs_v3
LOG_LEVEL=INFO
PORT=8000
MAX_CONTEXT_TOKENS=4000
CONTENT_VERSION=1.0
RATE_LIMIT_REQUESTS=100
RATE_LIMIT_WINDOW=60
SESSION_TIMEOUT_MINUTES=30
DEFAULT_CONFIDENCE_THRESHOLD=0.7
OPENAI_MODEL=gpt-4-turbo
COHERE_MODEL=embed-multilingual-v3.0
```

## Performance Metrics

The service provides performance monitoring with the following metrics:
- Response times
- Error rates
- Request volumes
- Cache hit rates
- Resource utilization

## Security Considerations

- All API endpoints require authentication
- Input sanitization is performed on all user inputs
- Rate limiting prevents abuse
- Session data is not permanently stored
- API keys should be kept secure

## Integration Examples

### JavaScript (Frontend)
```javascript
// Chat endpoint example
const response = await fetch('http://localhost:8000/api/chat', {
  method: 'POST',
  headers: {
    'Authorization': 'Bearer YOUR_API_KEY',
    'Content-Type': 'application/json'
  },
  body: JSON.stringify({
    query_text: 'What are the key components of humanoid robotics?'
  })
});

const data = await response.json();
console.log(data.response_text);
```

### WebSocket Example
```javascript
const ws = new WebSocket('ws://localhost:8000/ws/chat');

ws.onopen = function() {
  ws.send(JSON.stringify({
    query: 'What is Physical AI?',
    session_id: 'session123'
  }));
};

ws.onmessage = function(event) {
  const data = JSON.parse(event.data);
  if (data.type === 'response') {
    console.log('Response:', data.response);
  }
};
```

## Troubleshooting

### Common Issues
- **401 Unauthorized**: Check your API key
- **429 Too Many Requests**: Wait before making more requests
- **500 Internal Server Error**: Check server logs
- **Slow responses**: May be due to vector database queries

### Performance Tips
- Use sessions to maintain conversation context
- Leverage the caching system for repeated queries
- Monitor response times to identify performance bottlenecks
- Use WebSocket for real-time applications to reduce latency