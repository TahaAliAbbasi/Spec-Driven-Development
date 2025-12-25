# Implementation Plan: Phase 3 - Agent-Based Response Generation & Orchestration

**Feature**: Phase 3 - Agent-Based Response Generation & Orchestration
**Created**: 2025-12-13
**Status**: Draft
**Author**: Claude Code

## Technical Context

The Phase 3 implementation involves creating an agent-based response generation system that consumes ContextBundle objects from Phase 2 and generates constitutionally-compliant responses. The system uses OpenAI Agents SDK as an orchestration layer, while Gemini is the underlying language model provider. The system will be built with FastAPI and will enforce zero hallucination policy by strictly grounding responses in retrieved context.

The system architecture centers around a primary RAGAnswerAgent that receives user queries and ContextBundles, then generates responses with proper source citations. The agent will operate with deterministic behavior using fixed system prompts and constrained randomness. A provider adapter layer enables the use of Gemini models through the OpenAI Agents SDK interface.

**Known Unknowns**:
- Specific adapter implementation details for Gemini compatibility
- Exact token limits for response generation
- Detailed error handling strategies beyond basic requirements

## Constitution Check

This implementation must comply with the Global Constitution v2.0.0:

✅ **II. Zero Hallucination Policy**: The system will generate responses ONLY based on provided ContextBundles, with explicit refusal when context is insufficient.

✅ **III. Deterministic, Explainable AI Behavior**: The system will provide clear attribution to source material in all responses.

✅ **XII. Selected-Text-Only Answering**: The system will enforce selected-text-only mode when ContextBundle.status ≠ "success".

✅ **XIII. Fallback Responses**: The system will return explicit insufficiency responses when context is insufficient.

✅ **XIX. Mandatory Tech Stack Usage**: The system will use FastAPI for the backend as required.

## Gates

- [X] Constitutional compliance: All constitutional requirements are addressed
- [X] Scope alignment: Implementation aligns with Phase 3 requirements
- [X] Interface compatibility: System can consume ContextBundle objects from Phase 2
- [X] Architecture feasibility: OpenAI Agents SDK can implement required functionality

---

## Phase 0: Research & Analysis

### Research Tasks

1. **OpenAI Agents SDK Integration**
   - Research: How to implement RAG agents with OpenAI Agents SDK
   - Research: Best practices for deterministic agent behavior
   - Research: Proper citation handling in agent responses

2. **FastAPI Integration**
   - Research: FastAPI patterns for agent-based systems
   - Research: Error handling best practices for agent systems
   - Research: Request/response validation with Pydantic models

3. **Constitutional Compliance Implementation**
   - Research: Techniques for enforcing zero hallucination in agent responses
   - Research: Methods for ensuring deterministic behavior
   - Research: Source attribution patterns in RAG systems

### Decision Log

**Decision**: Use OpenAI Agents SDK as orchestration layer with Gemini as the underlying language model provider via a compatibility adapter
**Rationale**: This satisfies the requirement to use OpenAI Agents SDK for orchestration while using Gemini for text generation, with the adapter providing necessary API compatibility
**Alternatives considered**: Direct OpenAI model usage, other provider models

**Decision**: Implement deterministic behavior through fixed system prompts and temperature=0
**Rationale**: This ensures consistent outputs for identical inputs as required by the specification
**Alternatives considered**: Using random seeds, other constraint mechanisms

---

## Phase 1: Design & Architecture

### Data Models

#### AgentInput
- query: str - The user's question/query
- context_bundle: ContextBundle - The context bundle from Phase 2
- mode: str - Either "global" or "selected_text_only"

#### AgentOutput
- answer: str - The generated answer text
- citations: List[ChunkReference] - List of source citations
- used_chunks: List[str] - List of chunk IDs used in the response
- status: str - "answered" | "insufficient_context" | "refused"
- warnings: Optional[List[str]] - Optional warnings

#### ChunkReference
- chunk_id: str - ID of the referenced chunk
- source_url: str - URL of the source document

### API Contracts

#### POST `/api/answer`
**Request Body**:
```json
{
  "query": "str",
  "context_bundle": { /* ContextBundle from Phase 2 */ },
  "mode": "global"
}
```

**Response**:
```json
{
  "answer": "Generated answer text",
  "citations": [
    {
      "chunk_id": "chunk_123",
      "source_url": "https://example.com/source"
    }
  ],
  "used_chunks": ["chunk_123", "chunk_456"],
  "status": "answered",
  "warnings": ["optional warnings"]
}
```

**Error Responses**:
- 400: Invalid input format
- 422: Constitutional violation (e.g., context insufficient for selected-text-only mode)
- 500: Agent failure

### System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   User Query    │───▶│  RAGAnswerAgent │───▶│   Response      │
│   & Context     │    │                 │    │   with Citations│
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │
                       ┌─────────────────┐
                       │ Provider Adapter│
                       │ (OpenAI SDK -> │
                       │  Gemini API)    │
                       └─────────────────┘
                              │
                       ┌─────────────────┐
                       │   Gemini API    │
                       │ (gemini model)  │
                       └─────────────────┘
```

**Key Architecture Note**: OpenAI Agents SDK is used as an orchestration layer, while Gemini is the underlying language model provider. The provider adapter translates OpenAI Agents SDK calls to Gemini API calls, enabling compatibility between the orchestration framework and the model provider.

### Component Design

1. **RAGAnswerAgent**: Primary agent for response generation
   - Receives AgentInput
   - Processes context and query
   - Generates response with citations
   - Returns AgentOutput
   - **CRITICAL**: NO retrieval, memory, or tool discovery capabilities
   - **CRITICAL**: Single-pass generation with fixed prompts
   - **CRITICAL**: Temperature = 0 for deterministic behavior

2. **Provider Adapter**: Compatibility layer between orchestration and model
   - Translates OpenAI Agents SDK calls to Gemini API calls
   - Maintains API compatibility for agent orchestration
   - Handles authentication for Gemini API
   - **CRITICAL**: Enforces "NO RETRIEVAL" hard guard

3. **FastAPI Controller**: Handles API requests
   - Validates input
   - Orchestrates agent execution
   - Returns structured responses

4. **Constitutional Validator**: Ensures compliance
   - Checks context sufficiency
   - Validates selected-text-only mode based on request mode
   - Enforces zero hallucination policy
   - Implements explicit refusal rules
   - Ensures strict grounding per sentence

---

## Phase 2: Implementation Approach

### Development Strategy
1. Set up FastAPI application structure
2. Implement data models and validation
3. Create provider adapter for Gemini compatibility with OpenAI Agents SDK
4. Create RAGAnswerAgent with strict deterministic constraints
5. Implement constitutional compliance checks with explicit refusal rules
6. Implement "NO RETRIEVAL" hard guard
7. Add API endpoints with proper error handling
8. Add logging and observability
9. Create comprehensive tests

### Quality Assurance
- Unit tests for all core components
- Integration tests for API endpoints
- Constitutional compliance validation tests
- Determinism verification tests (identical inputs produce identical outputs)
- Zero hallucination validation tests
- Selected-text-only mode enforcement tests
- "NO RETRIEVAL" hard guard tests
- Performance testing for response times

### Security Considerations
- Input validation to prevent injection attacks
- Proper error handling to avoid information leakage
- Rate limiting for API endpoints
- Secure handling of API keys

### Performance Considerations
- Efficient context processing
- Caching for frequently accessed data
- Proper timeout handling for API calls
- Resource optimization for agent operations

## Re-evaluation of Constitution Check

After design completion, all constitutional requirements remain satisfied:
- Zero hallucination policy is enforced through context-bound generation with strict grounding per sentence
- Deterministic behavior is achieved through fixed prompts, temperature=0, and single-pass generation
- Selected-text-only mode is enforced by checking request mode and implementing explicit refusal rules
- Source attribution is implemented through citation tracking with deterministic chunk ordering
- "NO RETRIEVAL" hard guard prevents any fetching, searching, or inferring beyond provided context
- FastAPI backend aligns with required tech stack
- OpenAI Agents SDK is used as an orchestration layer, while Gemini is the underlying language model provider