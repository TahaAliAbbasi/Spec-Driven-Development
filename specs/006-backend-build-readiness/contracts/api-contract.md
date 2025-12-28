# API Contract: RAG Chatbot Backend Service

## Base URL
`http://localhost:8000` (or deployment URL)

## Authentication
Bearer token authentication using `FRONTEND_API_KEY` environment variable

## Endpoints

### Health Check
```
GET /api/health
```
**Description**: Health check endpoint to verify service status
**Response**:
```json
{
  "status": "healthy",
  "service": "rag-chatbot-backend",
  "timestamp": "2023-12-28T10:00:00.000Z"
}
```

### Root Endpoint
```
GET /
```
**Description**: Basic service information
**Response**:
```json
{
  "message": "RAG Chatbot Backend Service is running"
}
```

### Chat Endpoint
```
POST /api/chat
```
**Description**: Main chat endpoint managed by agent
**Headers**:
- `Authorization: Bearer {api_key}`
- `Content-Type: application/json`
**Request Body**:
```json
{
  "query_text": "user query text",
  "session_id": "optional session identifier"
}
```
**Response**:
```json
{
  "response_text": "generated response",
  "citations": [],
  "confidence_score": 0.85,
  "session_id": "session identifier"
}
```

### Retrieve Endpoint
```
POST /api/retrieve
```
**Description**: Retrieve context for backward compatibility
**Headers**:
- `Authorization: Bearer {api_key}`
- `Content-Type: application/json`
**Request Body**:
```json
{
  "query": "search query",
  "top_k": 5,
  "chapter_filter": "optional chapter filter",
  "section_filter": "optional section filter"
}
```
**Response**:
```json
{
  "retrieved_chunks": [
    {
      "content": "retrieved content",
      "source_url": "source URL",
      "chapter": "chapter name",
      "section": "section name",
      "chunk_id": "chunk identifier",
      "relevance_score": 0.95,
      "token_count": 120
    }
  ]
}
```

### Answer Endpoint
```
POST /api/answer
```
**Description**: Answer endpoint for backward compatibility
**Headers**:
- `Authorization: Bearer {api_key}`
- `Content-Type: application/json`
**Request Body**:
```json
{
  "query": "question text",
  "context_bundle": {
    "retrieved_chunks": []
  },
  "mode": "standard"
}
```
**Response**:
```json
{
  "response": "generated answer",
  "status": "success",
  "citations": [],
  "confidence_score": 0.85
}
```

### WebSocket Chat
```
WS /ws/chat
```
**Description**: WebSocket endpoint for real-time chat communication
**Message Format**:
```json
{
  "query": "user query",
  "session_id": "optional session id"
}
```
**Response Format**:
```json
{
  "type": "response|typing|error",
  "response": "response text",
  "citations": [],
  "confidence_score": 0.85,
  "session_id": "session id",
  "timestamp": "2023-12-28T10:00:00.000Z"
}
```

## Error Responses

All endpoints return standard error responses:
```json
{
  "detail": "error message"
}
```

**Common HTTP Status Codes**:
- `200`: Success
- `400`: Bad Request (invalid input)
- `401`: Unauthorized (invalid API key)
- `429`: Rate Limited (too many requests)
- `500`: Internal Server Error

## Rate Limiting

All endpoints are rate-limited based on configuration:
- Root endpoint: 150 requests per window
- Health check: 200 requests per window
- Chat endpoint: Configurable via RATE_LIMIT_REQUESTS
- Retrieve endpoint: 75 requests per window
- Answer endpoint: 60 requests per window