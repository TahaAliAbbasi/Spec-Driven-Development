# Implementation Tasks: Phase 2 - Retrieval & Context Assembly

## Feature Overview
This document outlines the implementation tasks for the Retrieval & Context Assembly system. The system will accept user queries or selected text, convert them to vector representations, and retrieve the most relevant content chunks from the Qdrant vector database based on cosine similarity. Implementation will be in a single file as requested.

## Phase 1: Setup
**Goal**: Initialize project structure and configure dependencies

- [X] T001 Create backend/retrieval directory structure
- [X] T002 add required dependencies (fastapi, uvicorn, cohere, qdrant-client, python-dotenv) through uv
- [X] T003 Create backend/retrieval/main.py as single implementation file
- [X] T004 Set up logging configuration for retrieval service

## Phase 2: Foundational Components
**Goal**: Implement core data models and utility functions needed by all user stories

- [X] T005 [P] Define RetrievalRequest Pydantic model in main.py
- [X] T006 [P] Define RetrievedChunk Pydantic model in main.py
- [X] T007 [P] Define ContextBundle Pydantic model in main.py
- [X] T008 [P] Implement token counting utility function in main.py using Phase 1 heuristic (words * 1.33)
- [X] T009 [P] Create error handling classes for retrieval-specific exceptions in main.py
- [X] T010 [P] Set up Cohere client initialization with API key from environment in main.py
- [X] T011 [P] Set up Qdrant client initialization with URL and API key from environment in main.py

## Phase 3: Core Vectorization Service [US1]
**Goal**: Implement the service that converts text to vector representations for similarity search

**Independent Test Criteria**: The service should successfully convert text to 1024-dimensional vectors using Cohere's embed-multilingual-v3.0 model, matching Phase 1 embedding dimensions.

- [X] T012 [US1] Implement vectorize_text function in main.py
- [X] T013 [US1] Add retry mechanism for Cohere API calls with exponential backoff in main.py
- [X] T014 [US1] Implement validation for input text length (max 1000 chars for query, 5000 for selected_text) in main.py
- [X] T015 [US1] Add error handling for Cohere API failures in main.py
- [X] T016 [US1] Write unit tests for vectorization functionality in main.py
- [X] T017 [US1] Implement environment variable validation for Cohere API in main.py

## Phase 4: Similarity Search Engine [US2]
**Goal**: Implement the engine that queries Qdrant vector database for relevant chunks

**Independent Test Criteria**: The engine should successfully perform similarity search against Qdrant and return top-k most relevant chunks based on cosine similarity.

- [X] T018 [US2] Implement search function in main.py to query Qdrant vector database
- [X] T019 [US2] Implement vector search using Qdrant's cosine similarity in main.py
- [X] T020 [US2] Apply top-k selection strategy with relevance threshold (0.3) in main.py
- [X] T021 [US2] Handle empty database and no matches scenarios in main.py
- [X] T022 [US2] Add error handling for Qdrant connection issues in main.py
- [X] T023 [US2] Implement payload filtering for metadata constraints in main.py
- [X] T024 [US2] Write unit tests for similarity search functionality in main.py

## Phase 5: Metadata Filtering [US3]
**Goal**: Implement the filtering layer that applies chapter/section/language constraints to search results

**Independent Test Criteria**: The filter should correctly apply metadata constraints (chapter, section, language, version) to search results and remove duplicates.

- [X] T025 [US3] Implement chapter-aware filtering with case-insensitive comparison in main.py
- [X] T026 [US3] Implement section-aware filtering with case-insensitive comparison in main.py
- [X] T027 [US3] Implement language and version constraint filtering in main.py
- [X] T028 [US3] Implement deduplication logic with relevance score prioritization in main.py
- [X] T029 [US3] Implement handling of near-duplicate chunks (>80% content overlap) in main.py
- [X] T030 [US3] Add metadata validation logic in main.py
- [X] T031 [US3] Implement filtering by multiple metadata criteria simultaneously in main.py
- [X] T032 [US3] Write unit tests for metadata filtering functionality in main.py

## Phase 6: Context Assembly [US4]
**Goal**: Implement the assembler that orders and bundles filtered chunks into final context within token limits

**Independent Test Criteria**: The assembler should create ordered context bundles within 4000 token limits, preserving chunk boundaries and metadata.

- [X] T033 [US4] Implement assemble_context function in main.py
- [X] T034 [US4] Implement token-aware assembly with relevance-based ordering in main.py
- [X] T035 [US4] Enforce maximum context size constraints (4000 tokens) in main.py
- [X] T036 [US4] Preserve chunk boundary and metadata during assembly in main.py
- [X] T037 [US4] Implement stable, deterministic ordering using relevance score and chunk_id in main.py
- [X] T038 [US4] Add secondary ranking heuristics (same chapter prioritization) in main.py
- [X] T039 [US4] Implement chunk truncation when token limits are exceeded in main.py
- [X] T040 [US4] Write unit tests for context assembly functionality in main.py

## Phase 7: Retrieval Controller [US5]
**Goal**: Implement the controller that orchestrates the entire retrieval process

**Independent Test Criteria**: The controller should handle the complete retrieval flow from request validation to response generation, supporting both query and selected-text modes.

- [X] T041 [US5] Implement retrieve_context function in main.py
- [X] T042 [US5] Integrate all services (vectorization, search, filtering, assembly) in main.py
- [X] T043 [US5] Implement retrieval modes: standard query and selected-text-only in main.py
- [X] T044 [US5] Add fallback behavior for invalid selected text in main.py
- [X] T045 [US5] Implement selected-text enforcement with semantic boundary validation in main.py
- [X] T046 [US5] Add processing time tracking and metadata generation in main.py
- [X] T047 [US5] Implement result validation to ensure zero hallucination in main.py
- [X] T048 [US5] Write unit tests for retrieval controller functionality in main.py

## Phase 8: API Endpoints [US6]
**Goal**: Expose retrieval functionality through FastAPI endpoints

**Independent Test Criteria**: The API should accept requests according to the contract and return properly formatted responses with correct status codes.

- [X] T049 [US6] Create FastAPI app instance in main.py
- [X] T050 [US6] Implement POST /api/retrieve endpoint with request/response validation in main.py
- [X] T051 [US6] Implement GET /api/health/retrieval endpoint in main.py
- [X] T052 [US6] Add request validation and error handling middleware in main.py
- [X] T053 [US6] Implement proper status code responses (200, 400, 500) in main.py
- [X] T054 [US6] Add error response formatting with error codes in main.py
- [X] T055 [US6] Implement request ID generation for traceability in main.py
- [X] T056 [US6] Write integration tests for API endpoints in main.py

## Phase 9: Error Handling & Edge Cases [US7]
**Goal**: Implement comprehensive error handling and edge case management

**Independent Test Criteria**: The system should gracefully handle all error scenarios with appropriate status messages and logging.

- [X] T057 [US7] Implement empty database detection and response in main.py
- [X] T058 [US7] Handle no relevant matches scenario with proper status in main.py
- [X] T059 [US7] Manage partial matches with confidence indicators in main.py
- [X] T060 [US7] Detect and handle conflicting metadata in retrieved chunks in main.py
- [X] T061 [US7] Implement scope violation detection for selected-text mode in main.py
- [X] T062 [US7] Add comprehensive error logging in main.py
- [X] T063 [US7] Implement retry logic for transient failures in main.py
- [X] T064 [US7] Write tests for all error handling scenarios in main.py

## Phase 10: Observability & Security [US8]
**Goal**: Implement logging, metrics, and security compliance features

**Independent Test Criteria**: The system should log all retrieval requests, track metrics, and comply with security requirements.

- [X] T065 [US8] Add comprehensive retrieval logging with timestamps and parameters in main.py
- [X] T066 [US8] Implement response time tracking for metrics in main.py
- [X] T067 [US8] Add chunk ID logging for traceability in main.py
- [X] T068 [US8] Ensure no raw embeddings are exposed in responses in main.py
- [X] T069 [US8] Implement no persistence of user queries beyond processing in main.py
- [X] T070 [US8] Add request ID for tracking individual operations in main.py
- [X] T071 [US8] Implement metric collection for query volume and success rates in main.py
- [X] T072 [US8] Write tests for observability features in main.py

## Phase 11: Polish & Cross-Cutting Concerns
**Goal**: Complete the implementation with final touches and cross-cutting concerns

- [X] T073 Add comprehensive documentation to all functions and classes in main.py
- [X] T074 Implement performance optimizations for retrieval speed in main.py
- [X] T075 Add configuration validation for environment variables in main.py
- [X] T076 Create health check validation for both Cohere and Qdrant connectivity in main.py
- [X] T077 Add request/response logging for debugging purposes in main.py
- [X] T078 Perform integration testing of complete retrieval pipeline in main.py
- [X] T079 Run performance tests to ensure retrieval under 1 second p95 in main.py
- [X] T080 Verify constitutional compliance (zero hallucination policy) in main.py
- [X] T081 Update README with retrieval service usage instructions

## Dependencies

**User Story Completion Order**:
1. US1 (Vectorization) → US2 (Search) → US3 (Filtering) → US4 (Assembly) → US5 (Controller) → US6 (API) → US7 (Error Handling) → US8 (Observability)

**Parallel Execution Examples**:
- P1: T005-T007 (data models) can be developed in parallel
- P2: T010-T011 (client setup) can be done in parallel with data models
- P3: T025-T032 (filtering) can be developed while other services are being built

## Implementation Strategy

**MVP Scope**: US1-US6 (Complete retrieval pipeline with API)
- Vectorization functionality in single file
- Similarity search engine in single file
- Basic filtering in single file
- Context assembly in single file
- Retrieval controller in single file
- API endpoints in single file

**Incremental Delivery**:
1. Phase 1-2: Foundation (models, utilities, setup) in single file
2. Phase 3-4: Core retrieval (vectorization, search) in single file
3. Phase 5-6: Assembly and API (filtering, assembly, endpoints) in single file
4. Phase 7-8: Advanced features (error handling, observability) in single file
5. Phase 9-11: Production readiness (polish, optimization) in single file