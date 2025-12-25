# Tasks: OpenRouter Integration with uv Package Manager

**Feature**: OpenRouter Integration for Agent-Based Response Generation
**Branch**: 1-openrouter-integration
**Generated**: 2025-12-16
**Based on**: spec.md, plan.md, data-model.md, research.md

## Implementation Strategy

**MVP Approach**: Implement OpenRouter integration with basic functionality first, then add advanced features. Start with the core OpenRouterProviderAdapter and integrate it with the existing RAGAnswerAgent.

**Priority Order**:
1. Setup and configuration (Phase 1-2)
2. Core OpenRouter integration (US1)
3. Configuration management (US2)
4. Error handling and fallbacks (US3)
5. Testing and validation (US4)
6. Polish and documentation (Final Phase)

## Dependencies

- **User Story 2** depends on **User Story 1** (Configuration requires working OpenRouter integration)
- **User Story 3** depends on **User Story 1** (Error handling requires working OpenRouter integration)
- **User Story 4** depends on **User Story 1, 2, 3** (Testing requires all functionality)

## Parallel Execution Examples

**Within US1 (Core Integration)**:
- T015 [P] [US1] Create OpenRouterProviderAdapter class in backend/response_generation/agents/openrouter_provider_adapter.py
- T016 [P] [US1] Implement generate_response method in backend/response_generation/agents/openrouter_provider_adapter.py
- T017 [P] [US1] Implement extract_citations_from_response method in backend/response_generation/agents/openrouter_provider_adapter.py

**Within US2 (Configuration)**:
- T025 [P] [US2] Update config.py to include OpenRouter settings in backend/response_generation/config.py
- T026 [P] [US2] Update .env to use OpenRouter variables in backend/response_generation/.env

## Phase 1: Setup Tasks

### Goal
Initialize the project with uv package manager and set up the basic structure for OpenRouter integration.

- [x] T001 Install uv package manager on development environment
- [x] T002 Create pyproject.toml based on existing requirements.txt in backend/response_generation/pyproject.toml
- [x] T003 Add dependencies to pyproject.toml using uv add command in backend/response_generation/pyproject.toml
- [x] T004 Generate uv.lock file for deterministic builds in backend/response_generation/uv.lock
- [x] T005 Remove old requirements.txt Google Generative AI dependency in backend/response_generation/requirements.txt
- [x] T006 Update README to reflect uv usage in backend/response_generation/README.md

## Phase 2: Foundational Tasks

### Goal
Prepare the foundation for OpenRouter integration by updating configuration and removing old Gemini code.

- [x] T010 Update config.py to remove Gemini settings in backend/response_generation/config.py
- [x] T011 Create OpenRouter-specific environment variables in backend/response_generation/.env
- [x] T012 Remove old provider_adapter.py file in backend/response_generation/agents/provider_adapter.py
- [x] T013 Update agents/__init__.py to remove ProviderAdapter reference in backend/response_generation/agents/__init__.py
- [x] T014 Update debug_test.py to use new OpenRouter adapter in backend/response_generation/debug_test.py

## Phase 3: [US1] Core OpenRouter Integration

### Goal
Implement the core OpenRouterProviderAdapter with API integration and response generation functionality.

### Independent Test Criteria
- Can successfully make API calls to OpenRouter
- Can generate responses based on context and query
- Can extract citations from responses
- Maintains constitutional compliance (no hallucinations)

- [x] T015 [P] [US1] Create OpenRouterProviderAdapter class in backend/response_generation/agents/openrouter_provider_adapter.py
- [x] T016 [P] [US1] Implement OpenRouterProviderAdapter.__init__ method with configuration validation in backend/response_generation/agents/openrouter_provider_adapter.py
- [x] T017 [P] [US1] Implement generate_response method with OpenRouter API call in backend/response_generation/agents/openrouter_provider_adapter.py
- [x] T018 [US1] Implement extract_citations_from_response method in backend/response_generation/agents/openrouter_provider_adapter.py
- [x] T019 [US1] Implement validate_no_retrieval_capability method in backend/response_generation/agents/openrouter_provider_adapter.py
- [x] T020 [US1] Test OpenRouterProviderAdapter with mock data in backend/response_generation/agents/openrouter_provider_adapter.py

## Phase 4: [US2] Configuration Management

### Goal
Update configuration system to use OpenRouter-specific settings and environment variables.

### Independent Test Criteria
- Configuration loads OpenRouter API key correctly
- Configuration validates required settings
- Configuration supports configurable model selection

- [x] T025 [P] [US2] Update config.py to include OpenRouter settings in backend/response_generation/config.py
- [x] T026 [P] [US2] Add OPENROUTER_API_KEY validation in backend/response_generation/config.py
- [x] T027 [US2] Add OPENROUTER_BASE_URL and OPENROUTER_MODEL configuration in backend/response_generation/config.py
- [x] T028 [US2] Update .env to use OpenRouter variables in backend/response_generation/.env
- [x] T029 [US2] Test configuration validation with missing API key in backend/response_generation/config.py

## Phase 5: [US3] Error Handling and Fallbacks

### Goal
Implement proper error handling for OpenRouter API failures and edge cases.

### Independent Test Criteria
- Handles API connection errors gracefully
- Provides appropriate error messages when OpenRouter is unavailable
- Implements timeout mechanisms for API calls

- [x] T035 [P] [US3] Add API connection error handling in backend/response_generation/agents/openrouter_provider_adapter.py
- [x] T036 [P] [US3] Implement timeout handling for OpenRouter API calls in backend/response_generation/agents/openrouter_provider_adapter.py
- [x] T037 [US3] Add retry logic for failed API calls in backend/response_generation/agents/openrouter_provider_adapter.py
- [x] T038 [US3] Test error handling with invalid API key in backend/response_generation/agents/openrouter_provider_adapter.py

## Phase 6: [US4] Agent Integration and Testing

### Goal
Integrate OpenRouter provider with RAGAnswerAgent and ensure all functionality works properly.

### Independent Test Criteria
- RAGAnswerAgent uses OpenRouter provider successfully
- All constitutional compliance requirements are maintained
- Citations are properly tracked
- System refuses to answer when context is insufficient

- [x] T045 [P] [US4] Update RAGAnswerAgent to use OpenRouterProviderAdapter in backend/response_generation/agents/rag_answer_agent.py
- [x] T046 [P] [US4] Test RAGAnswerAgent with OpenRouter integration in backend/response_generation/agents/rag_answer_agent.py
- [x] T047 [US4] Verify constitutional compliance with OpenRouter responses in backend/response_generation/agents/rag_answer_agent.py
- [x] T048 [US4] Test citation tracking with OpenRouter responses in backend/response_generation/agents/rag_answer_agent.py
- [x] T049 [US4] Test insufficient context handling with OpenRouter in backend/response_generation/agents/rag_answer_agent.py

## Phase 7: [US5] API and Endpoint Validation

### Goal
Ensure the API endpoints work properly with the new OpenRouter integration.

### Independent Test Criteria
- API endpoints return proper responses
- Error handling works at the API level
- All acceptance scenarios from spec are satisfied

- [x] T055 [P] [US5] Test /api/answer endpoint with OpenRouter integration in backend/response_generation/main.py
- [x] T056 [P] [US5] Verify error handling at API level in backend/response_generation/main.py
- [x] T057 [US5] Test all acceptance scenarios from spec with OpenRouter in backend/response_generation/test_api.py
- [x] T058 [US5] Update test_basic.py for OpenRouter integration in backend/response_generation/test_basic.py

## Final Phase: Polish & Cross-Cutting Concerns

### Goal
Complete the implementation with documentation, cleanup, and final validation.

- [x] T090 Update documentation to reflect OpenRouter integration in backend/response_generation/README.md
- [x] T091 Create quickstart guide for uv and OpenRouter in specs/1-openrouter-integration/quickstart.md
- [x] T092 Update agent context with new technologies in .specify/memory/agent-context.md
- [x] T093 Run full test suite with uv package manager in backend/response_generation/
- [x] T094 Verify all constitutional compliance requirements are met in backend/response_generation/
- [x] T095 Update CI/CD pipeline to use uv instead of pip in .github/workflows/
- [x] T096 Clean up any remaining Gemini references in codebase
- [x] T097 Perform final integration test with full workflow