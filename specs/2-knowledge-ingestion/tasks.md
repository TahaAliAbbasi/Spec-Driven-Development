# Tasks: Phase 1 - Knowledge Ingestion & Vectorization

**Feature**: 2-knowledge-ingestion
**Created**: 2025-12-13
**Status**: Draft

## Overview

This document contains the implementation tasks for Phase 1: Knowledge Ingestion & Vectorization. The system will discover, extract, chunk, embed, and store book content in Qdrant for RAG functionality.

**Target URL**: https://physical-ai-and-humanoid-robotics-lemon.vercel.app/
**SiteMap URL**: https://physical-ai-and-humanoid-robotics-lemon.vercel.app/sitemap.xml
## Phase 1: Setup

**Goal**: Initialize project with required dependencies and configuration

**Independent Test**: Project structure can be created and dependencies installed successfully

- [X] T001 Create backend directory structure
- [X] T002 Initialize Python project with UV package manager
- [X] T003 Install required dependencies: cohere, qdrant-client, requests, beautifulsoup4, python-dotenv
- [X] T004 Create .env file with environment variable placeholders
- [X] T005 Create main.py file with basic structure and imports

## Phase 2: Foundational Components

**Goal**: Implement core components needed by all user stories

**Independent Test**: Core components can be imported and basic functionality verified

- [X] T006 [P] Setup Cohere client with API key from environment
- [X] T007 [P] Setup Qdrant client with API key and URL from environment
- [X] T008 [P] Create content extraction helper functions using requests and BeautifulSoup
- [X] T009 [P] Create token counting function for chunking logic
- [X] T010 [P] Create logging configuration for observability
- [X] T011 [P] Create retry mechanism with exponential backoff for API calls

## Phase 3: User Story 1 - Book Content Ingestion (Priority: P1)

**Goal**: System automatically discovers and extracts clean textual content from the deployed book website

**Independent Test**: Can be fully tested by running the ingestion pipeline on a book URL and verifying that clean text content is extracted and stored in the vector database

- [X] T012 [US1] Implement get_all_urls() function to discover all pages from target website
- [X] T013 [US1] Implement extract_text_from_urls() function to extract clean text from each URL
- [X] T014 [US1] Implement content cleaning logic to remove navigation, footer, ads, and boilerplate elements
- [X] T015 [US1] Preserve semantic structure including headings, code blocks, and chapter/section boundaries
- [X] T016 [US1] Add error handling for unreachable URLs during discovery
- [X] T017 [US1] Add validation to ensure all discovered content is properly extracted

## Phase 4: User Story 2 - Content Chunking and Embedding (Priority: P1)

**Goal**: System chunks the extracted content using a deterministic strategy and generates embeddings for the content

**Independent Test**: Can be tested by providing text input and verifying that consistent chunking and embeddings are produced across multiple runs

- [X] T018 [US2] Implement chunk_text() function with 512 token chunks and 10% overlap (51 tokens)
- [X] T019 [US2] Generate deterministic chunk IDs for reproducible results across runs
- [X] T020 [US2] Implement embed() function using Cohere's embed-multilingual-v3.0 model
- [X] T021 [US2] Add batch processing for embedding generation with appropriate sizing
- [X] T022 [US2] Implement rate limiting handling for Cohere API calls
- [X] T023 [US2] Add failure recovery strategy for embedding generation

## Phase 5: User Story 3 - Vector Storage and Metadata (Priority: P2)

**Goal**: System stores the generated embeddings in the vector database with appropriate metadata for retrieval

**Independent Test**: Can be tested by verifying that vectors and metadata are correctly stored in the vector database and can be retrieved by ID

- [X] T024 [US3] Implement create_collection() function to initialize 'physical_ai_humanoid_docs_v1' collection
- [X] T025 [US3] Configure Qdrant collection with 1024-dim vectors and cosine distance
- [X] T026 [US3] Implement save_chunk_to_qdrant() function to store embeddings with metadata
- [X] T027 [US3] Ensure all required metadata fields (source_url, chapter, section, chunk_id, language, version) are stored
- [X] T028 [US3] Implement data integrity validation before insertion
- [X] T029 [US3] Implement deduplication strategy to prevent duplicate entries

## Phase 6: Integration and Main Pipeline

**Goal**: Connect all components into a complete, executable pipeline

**Independent Test**: Complete pipeline executes successfully from URL discovery to vector storage

- [X] T030 [P] Implement main() function to execute the complete pipeline
- [X] T031 [P] Add command-line argument parsing for configuration
- [X] T032 [P] Implement idempotent re-runs to allow safe re-execution without duplicating content
- [X] T033 [P] Add comprehensive logging and metrics for observability
- [X] T034 [P] Add validation to verify 100% of discoverable book pages are processed
- [X] T035 [P] Implement error handling for all edge cases (network failures, rate limits, etc.)

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Complete the implementation with production-ready features

**Independent Test**: Complete pipeline meets all success criteria and is ready for deployment

- [X] T036 Add comprehensive error logging and monitoring
- [X] T037 Implement graceful degradation when content is unavailable
- [X] T038 Add configuration options for chunk size and other parameters
- [X] T039 Ensure forward compatibility with OpenAI Agents SDK for later phases
- [X] T040 Add performance monitoring to ensure processing completes within 30 minutes
- [X] T041 Verify 95%+ boilerplate removal while preserving 100% of actual book content
- [X] T042 Test idempotent re-runs to ensure zero duplicate entries
- [X] T043 Document the complete pipeline for future maintenance

## Dependencies

User Story 2 (Content Chunking and Embedding) requires User Story 1 (Book Content Ingestion) to be completed first as it depends on the extracted content.

User Story 3 (Vector Storage and Metadata) requires both User Story 1 and 2 to be completed as it needs the chunked content with embeddings.

## Parallel Execution Examples

- Tasks T006-T011 can be executed in parallel as they implement independent foundational components
- Tasks T012-T017 can be developed in parallel with T018-T023 if mock data is used for integration testing
- Tasks T024-T029 can be developed in parallel with the other user stories using mock data

## Implementation Strategy

1. **MVP Scope**: Complete Phase 1 (Setup) and Phase 2 (Foundational) plus User Story 1 (Book Content Ingestion) for a minimal working pipeline
2. **Incremental Delivery**: Add chunking and embedding functionality, then vector storage
3. **Complete Pipeline**: Integrate all components and add polish features