# Feature Specification: OpenRouter Integration for Agent-Based Response Generation

## Overview
Replace the current Gemini LLM implementation with OpenRouter for Phase 3 - Agent-Based Response Generation & Orchestration. This change addresses the issue where the Gemini integration is no longer working and implements OpenRouter as the new LLM provider with a generated API key. The system will use OpenRouter to access OpenAI-compatible GPT models (gpt-4.1, gpt-4o, gpt-3.5-turbo) while maintaining native compatibility with OpenAI Agents SDK.

## User Scenarios & Testing

### Primary Scenario
1. As a user interacting with the agent system, I want my queries to be processed through OpenRouter GPT models instead of Gemini
2. As a system administrator, I want to configure OpenRouter API keys and model selection securely
3. As a developer, I want the backend response generation module to seamlessly integrate with OpenRouter using the OpenAI Agents SDK
4. As a user, I want the system to provide accurate answers based only on provided context without hallucinations

### Acceptance Scenarios
- [ ] User queries are successfully processed through OpenRouter API using GPT-class models
- [ ] System handles OpenRouter API authentication with configured API key
- [ ] Response quality remains consistent or improves compared to previous Gemini implementation
- [ ] Error handling works appropriately when OpenRouter API is unavailable
- [ ] Existing agent orchestration functionality continues to work with new LLM provider
- [ ] The system enforces zero hallucination by using only provided ContextBundle
- [ ] Agent responses are deterministic with temperature=0
- [ ] System refuses to answer when context is insufficient
- [ ] Citations are properly tracked from used chunks in responses

## Functional Requirements

### FR-1: OpenRouter API Integration
- The system shall connect to OpenRouter API endpoints using the configured API key
- The system shall support configurable model selection from OpenRouter's available GPT-class models
- The system shall handle API rate limiting and retry logic appropriately
- The system shall use OpenAI Agents SDK as the orchestration framework, configured to communicate with OpenRouter via custom base_url

### FR-2: OpenRouter Provider Adapter
- The system shall implement OpenRouterProviderAdapter with OpenAI-compatible request/response format
- The system shall include OpenRouter authentication headers in API requests
- The system shall support model routing via environment variables
- The system shall enforce deterministic generation with temperature = 0
- The system shall include explicit timeout and retry handling

### FR-3: RAG Agent Behavior
- The RAGAnswerAgent must use GPT-class models via OpenRouter
- The agent must enforce zero hallucination and generate answers strictly from the provided ContextBundle
- The agent must refuse to answer when context is insufficient
- The agent must track used chunks for citation purposes
- The agent must operate in single-pass generation mode only

### FR-4: Constitutional Compliance
- The system must enforce that no retrieval happens inside the LLM
- The model must never fetch, infer, or assume external knowledge
- All answers must be grounded solely in ContextBundle content
- The system must use fixed system prompt with temperature = 0
- The system must provide structured refusal when context is insufficient
- The system must refuse when ContextBundle.status ≠ "success"

### FR-5: Backend Response Generation Module Update
- The system shall update the `backend/response_generation` module to use OpenRouter instead of Gemini
- The system shall maintain backward compatibility with existing agent orchestration interfaces
- The system shall support the same response format and structure as before
- The system shall remove all Gemini-specific logic, code, adapters, or configuration

### FR-6: Configuration Management
- The system shall securely store and retrieve the OPENROUTER_API_KEY
- The system shall support OPENROUTER_BASE_URL=https://openrouter.ai/api/v1
- The system shall support configurable OPENROUTER_MODEL (e.g., gpt-4.1-mini) via environment variables
- The system shall not allow hardcoded model names in code

### FR-7: Error Handling and Fallbacks
- The system shall gracefully handle OpenRouter API failures
- The system shall provide appropriate error messages when OpenRouter is unavailable
- The system shall implement timeout mechanisms for API calls

## Success Criteria

### Quantitative Measures
- 99% of user queries successfully processed through OpenRouter API using GPT models
- Response times remain under 5 seconds for 95% of requests
- System maintains 99.5% uptime after the transition
- Zero hallucination rate achieved (100% of responses grounded in provided context)
- Deterministic outputs maintained (identical inputs produce identical outputs)

### Qualitative Measures
- Users experience seamless transition with no noticeable difference in response quality
- Developers can easily configure and manage OpenRouter integration with environment variables
- System demonstrates improved reliability compared to previous Gemini implementation
- OpenAI Agents SDK runs without compatibility hacks
- System is ready for Phase 4 frontend integration

## Key Entities
- OpenRouter API Configuration
- API Key Management
- Response Generation Pipeline
- Agent Orchestration Interface
- OpenRouterProviderAdapter
- ContextBundle
- RAGAnswerAgent
- OpenAI Agents SDK

## Assumptions
- OpenRouter API provides OpenAI-compatible endpoints for GPT-class models
- OpenRouter offers reliable service with appropriate rate limits for the application's needs
- The existing response generation architecture can accommodate OpenRouter with minimal changes
- The API key for OpenRouter has been properly generated and secured
- The OpenAI Agents SDK can be configured to work with OpenRouter's API endpoints

## Dependencies
- OpenRouter account and API access
- Existing agent orchestration infrastructure
- Backend infrastructure supporting the response generation module
- ContextBundle containing relevant information for user queries

## Constraints
- Must maintain existing API contracts with other system components
- Must not break existing agent orchestration functionality
- Must implement secure handling of API keys
- Must not modify Phase 2 retrieval logic
- Must not introduce Next.js, React, or a new frontend
- Must not embed API keys in frontend code
- Must remove all Gemini or Gemini SDK usage
- Model selection must be configurable via environment variables only