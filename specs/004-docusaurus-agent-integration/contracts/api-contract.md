# API Contract: Docusaurus Agent Integration

## Frontend to Backend API Calls

### Query Submission Endpoint
**Path**: `/api/answer` (consumes from Phase 3 backend)
**Method**: POST
**Content-Type**: application/json

#### Request Body
```json
{
  "query": "string, the user's question",
  "mode": "string, query mode (global or selected_text_only)",
  "context": {
    "selected_text": "string, optional selected text from book",
    "document_context": "array of strings, optional document context"
  }
}
```

#### Response Schema
```json
{
  "status": {
    "type": "string",
    "enum": ["ANSWERED", "INSUFFICIENT_CONTEXT", "REFUSED"],
    "description": "Status of the response"
  },
  "answer": {
    "type": "string",
    "description": "The AI-generated response text"
  },
  "citations": {
    "type": "array",
    "items": {
      "type": "object",
      "properties": {
        "chunk_id": {
          "type": "string",
          "description": "Unique identifier for the content chunk"
        },
        "source_url": {
          "type": "string",
          "format": "uri",
          "description": "URL to the original content location"
        }
      }
    },
    "description": "Array of citations referencing source material"
  },
  "metadata": {
    "type": "object",
    "properties": {
      "processing_time": {
        "type": "number",
        "description": "Time taken to process the query in milliseconds"
      },
      "confidence": {
        "type": "number",
        "minimum": 0,
        "maximum": 1,
        "description": "Confidence score of the response"
      }
    }
  }
}
```

### Error Response Schema
```json
{
  "error": {
    "type": "string",
    "description": "Error message describing what went wrong"
  },
  "status_code": {
    "type": "number",
    "description": "HTTP status code"
  },
  "timestamp": {
    "type": "string",
    "format": "date-time",
    "description": "When the error occurred"
  }
}
```

## Frontend Components API

### QueryInput Component
**Props**:
- onSubmit: function, callback when query is submitted
- placeholder: string, placeholder text for input field
- disabled: boolean, whether input is disabled

**Events**:
- onQueryChange: emitted when query text changes
- onQuerySubmit: emitted when query is submitted

### ResponseViewer Component
**Props**:
- response: object, the AgentOutput response from backend
- loading: boolean, whether response is still loading
- error: string, error message if any

### CitationsPanel Component
**Props**:
- citations: array, array of citation objects
- visible: boolean, whether panel is visible

### ErrorNotifier Component
**Props**:
- message: string, error message to display
- type: string, type of notification (error, warning, info)
- visible: boolean, whether notifier is visible
- onClose: function, callback when notifier is closed