# Data Model: RAG Chatbot Backend Integration

## Entity Design

### User Query
**Description**: The text input from the user in the floating chatbot, containing questions or requests for information

**Fields**:
- `query_text` (string): The actual text of the user's query
- `session_id` (string, optional): ID of the current conversation session (for multi-turn interactions)
- `timestamp` (datetime): When the query was submitted
- `user_id` (string, optional): Identifier for the user (if authenticated)

**Validation**:
- Length: 1-1000 characters
- Required: query_text

### Retrieved Context
**Description**: Relevant document chunks retrieved from the Qdrant vector database based on semantic similarity to the user query

**Fields**:
- `content` (string): The actual content of the retrieved chunk
- `source_url` (string): URL where the content originated
- `chapter` (string): Chapter or section identifier
- `section` (string): Specific section identifier
- `chunk_id` (string): Unique identifier for this chunk
- `relevance_score` (float): Similarity score between 0.0 and 1.0
- `token_count` (int): Number of tokens in the content

**Validation**:
- relevance_score: 0.0-1.0
- token_count: positive integer

### Generated Response
**Description**: AI-generated text response that answers the user's query based on the retrieved context

**Fields**:
- `response_text` (string): The actual response text
- `confidence_score` (float): Confidence level of the response (0.0-1.0)
- `citations` (list of Citation objects): References to source documents
- `timestamp` (datetime): When the response was generated
- `session_id` (string, optional): ID of the conversation session

**Validation**:
- confidence_score: 0.0-1.0
- Required: response_text

### Citation
**Description**: Reference to the source document or section that supports information in the generated response

**Fields**:
- `source_url` (string): URL of the source document
- `chapter` (string): Chapter identifier
- `section` (string): Section identifier
- `content_snippet` (string): Brief excerpt from the source
- `relevance_score` (float): How relevant this citation is to the response

**Validation**:
- relevance_score: 0.0-1.0

### Conversation Session
**Description**: Tracks the state of a conversation within a single user session

**Fields**:
- `session_id` (string): Unique identifier for the conversation session
- `created_at` (datetime): When the session started
- `last_interaction` (datetime): When the last message was exchanged
- `messages` (list of Message objects): History of messages in this session

### Message
**Description**: A single message in a conversation session

**Fields**:
- `role` (string): "user" or "assistant"
- `content` (string): The text of the message
- `timestamp` (datetime): When the message was created
- `metadata` (dict, optional): Additional information about the message

## Relationships

1. **User Query** → **Retrieved Context** (one-to-many): A query can result in multiple context chunks
2. **Retrieved Context** → **Generated Response** (many-to-one): Multiple context chunks contribute to one response
3. **Generated Response** → **Citation** (one-to-many): A response can have multiple citations
4. **Conversation Session** → **Message** (one-to-many): A session contains multiple messages

## State Transitions

### Conversation Session States
- `ACTIVE`: Session is currently accepting new messages
- `INACTIVE`: Session is idle but still valid
- `EXPIRED`: Session has timed out and is no longer valid

### Message States
- `PENDING`: Message is being processed
- `COMPLETED`: Message processing is finished
- `FAILED`: Message processing failed