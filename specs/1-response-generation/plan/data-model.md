# Data Models: Phase 3 - Agent-Based Response Generation

**Feature**: Phase 3 - Agent-Based Response Generation & Orchestration
**Created**: 2025-12-13
**Status**: Draft

## Core Entities

### AgentInput
Represents the input to the RAG agent from the API layer.

**Fields**:
- `query`: str - The user's question or query string
- `context_bundle`: ContextBundle - The context bundle object from Phase 2 retrieval
- `mode`: str - Either "global" or "selected_text_only" to determine response behavior

**Validation Rules**:
- Query must not be empty
- ContextBundle must be properly formatted with required fields
- Mode must be one of the allowed values
- When mode is "selected_text_only", additional constitutional validation is required based on context_bundle.status and content relevance

### AgentOutput
Represents the output from the RAG agent to the API layer.

**Fields**:
- `answer`: str - The generated answer text
- `citations`: List[ChunkReference] - List of source citations for the answer
- `used_chunks`: List[str] - List of chunk IDs that were used to generate the answer
- `status`: str - "answered" | "insufficient_context" | "refused"
- `warnings`: Optional[List[str]] - Optional list of warnings about the response

**Validation Rules**:
- Answer must be present when status is "answered"
- Citations must correspond to actual chunks in the context bundle when present
- Status must be one of the allowed values

### ChunkReference
Represents a citation to a specific chunk in the knowledge base.

**Fields**:
- `chunk_id`: str - Unique identifier of the referenced chunk
- `source_url`: str - URL or identifier of the source document

**Validation Rules**:
- Both fields must be present and non-empty
- chunk_id must match an actual chunk in the provided context bundle

### RAGAnswerAgent
The primary agent responsible for context-grounded response generation.

**Responsibilities**:
- Process user queries with provided context
- Generate responses grounded in provided context
- Create proper citations for all information in responses
- Enforce constitutional compliance (zero hallucination, etc.)
- Return appropriate status based on context sufficiency

**State Transitions**:
- Idle → Processing (when receiving input)
- Processing → Generating (when creating response)
- Generating → Complete (when response is ready)

## Relationships

- AgentInput contains one ContextBundle (from Phase 2)
- AgentOutput contains multiple ChunkReference objects (citations)
- AgentOutput references multiple chunk IDs from the original ContextBundle
- RAGAnswerAgent processes AgentInput and produces AgentOutput