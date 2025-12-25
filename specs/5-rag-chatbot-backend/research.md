# Research Findings: RAG Chatbot Backend Integration

## Research Summary

### 1. OpenAI Agent SDK Implementation

**Decision**: Use OpenAI Assistant API to create an agent that coordinates the RAG pipeline
**Rationale**: The OpenAI Assistant API provides built-in capabilities for managing conversations, tool calling, and orchestration of multiple services
**Alternatives considered**:
- Custom agent framework
- LangChain agents
- CrewAI agents

**Key findings**:
- OpenAI Assistant API allows creating assistants with custom instructions
- Supports tool calling to interact with external services (retrieval, response generation)
- Handles conversation state management
- Can be integrated with existing services via API calls

### 2. Integration Architecture

**Decision**: Create an agent controller that orchestrates existing services without replacing them
**Rationale**: This preserves existing tested functionality while adding agent coordination
**Alternatives considered**:
- Complete rewrite of all services
- Direct API integration without agent
- Microservice architecture with message queue

**Architecture approach**:
- Main agent in main.py coordinates the RAG flow
- Agent calls existing retrieval service to get context
- Agent calls existing response generation service
- Agent manages conversation state and user interactions
- Existing services remain unchanged for backward compatibility

### 3. uv Package Management

**Decision**: Use uv as the primary package manager for the project
**Rationale**: uv is a fast Python package manager that is compatible with existing pip-based workflows
**Key findings**:
- uv supports pyproject.toml and requirements.txt
- Faster than pip for dependency resolution
- Compatible with virtual environments
- Commands: `uv pip install`, `uv run`, etc.

### 4. Service Orchestration

**Decision**: Implement a coordinator pattern where the agent orchestrates existing services
**Rationale**: Maintains modularity while providing unified interface
**Key findings**:
- Agent acts as a facade pattern over existing services
- Existing endpoints remain available for direct access
- Agent handles the RAG flow: user input → retrieval → response generation → output
- Error handling and fallbacks managed at the agent level

## Implementation Strategy

### Agent Responsibilities
1. Receive user queries from frontend
2. Call retrieval service to get relevant context from Qdrant
3. Call response generation service to create answers based on context
4. Format responses with proper citations
5. Manage conversation state within session
6. Handle errors and fallback responses

### Integration Points
1. Connect to existing retrieval service API
2. Connect to existing response generation service API
3. Use existing .env configuration for API keys
4. Maintain existing Qdrant connection patterns
5. Preserve existing Cohere embedding patterns

### Technology Stack
- Python 3.11+
- uv package manager
- FastAPI for web framework
- OpenAI Agent SDK (Assistant API)
- Existing: Cohere, Qdrant, Pydantic models