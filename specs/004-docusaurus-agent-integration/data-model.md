# Data Model: Docusaurus Agent Integration

## Entities

### QueryInput
**Description**: User-provided text for question answering
**Fields**:
- queryText: string - The user's question or text input
- queryType: string - Type of query (e.g., "global", "selected_text_only")
- validationRules:
  - Must not be empty
  - Must be less than 1000 characters
  - Should not contain malicious content (XSS prevention)

### AgentOutput
**Description**: Response from Phase 3 backend containing status, answer text, and citations
**Fields**:
- status: string - Status of the response (ANSWERED, INSUFFICIENT_CONTEXT, REFUSED)
- answer: string - The AI-generated response text
- citations: array of Citation objects - References to source material
- metadata: object - Additional response metadata

### Citation
**Description**: Source reference containing chunk ID and URL for traceability to original content
**Fields**:
- chunkId: string - Unique identifier for the content chunk
- sourceUrl: string - URL to the original content location
- textPreview: string - Preview of the cited text (optional)

### UserFeedback
**Description**: Error messages and status notifications displayed to users during various scenarios
**Fields**:
- type: string - Type of feedback (SUCCESS, ERROR, WARNING, INFO)
- message: string - The feedback message to display
- timestamp: datetime - When the feedback was generated
- relatedAction: string - The action that triggered the feedback

## State Transitions

### Query Processing State Machine
1. **IDLE**: Initial state, waiting for user input
2. **LOADING**: Query submitted, waiting for backend response
3. **SUCCESS**: Valid response received from backend
4. **ERROR**: Error occurred during processing
5. **INSUFFICIENT_CONTEXT**: Backend returned insufficient context response
6. **REFUSED**: Backend refused to answer based on constitutional rules

## Validation Rules

### Input Validation
- Query text must be between 1 and 1000 characters
- Query text must not contain potentially harmful content
- Query type must be one of the allowed values

### Response Validation
- AgentOutput must contain a valid status
- Citations must have valid chunk IDs and URLs when present
- Answer text must not be empty when status is ANSWERED

## Relationships
- QueryInput is sent to the backend API
- AgentOutput contains multiple Citation objects
- UserFeedback is associated with specific query processing attempts