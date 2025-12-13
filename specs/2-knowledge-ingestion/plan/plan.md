# Implementation Plan: Phase 1 - Knowledge Ingestion & Vectorization

**Feature**: 2-knowledge-ingestion
**Created**: 2025-12-13
**Status**: Draft
**Plan Version**: 1.0.0

## Technical Context

**Problem Statement**: The system needs to ingest content from the deployed book website, chunk it, generate embeddings using Cohere, and store the vectors in Qdrant for RAG functionality.

**Target URL**: https://physical-ai-and-humanoid-robotics-lemon.vercel.app/

**Core Components**:
- URL discovery and validation
- Content extraction and cleaning
- Text chunking with semantic boundaries
- Embedding generation using Cohere
- Vector storage in Qdrant Cloud

**Resolved Parameters**:
- Cohere model: embed-multilingual-v3.0
- Chunk size: 512 tokens with 10% overlap (51 tokens)
- Qdrant collection: physical_ai_humanoid_docs_v1 with 1024-dim vectors

## Constitution Check

This implementation must comply with the Global Constitution:

✅ **Spec-Driven Development**: Following the spec created in the previous step
✅ **Zero Hallucination Policy**: Will only process actual book content
✅ **Deterministic Behavior**: Using deterministic chunking and embedding processes
✅ **Mandatory Tech Stack**: Using Cohere for embeddings and Qdrant Cloud for vector storage
✅ **RAG Rules**: Following chunking strategy and embedding rules per constitution

## Gates

- [x] Feature specification exists and is approved
- [x] Constitution compliance verified
- [x] Technology stack aligned with constitution
- [x] Unknowns resolved via research

## Phase 0: Research & Resolution of Unknowns

### Research Tasks

1. **Cohere Model Selection**: Research which Cohere embedding model is most appropriate for book content
2. **Chunking Parameters**: Determine optimal chunk size and overlap for book content
3. **Qdrant Configuration**: Define collection schema and configuration for physical_ai_humanoid_docs_v1

### Expected Outcomes

- Cohere model decision with rationale
- Chunking parameters optimized for book content
- Qdrant collection schema defined

## Phase 1: Design & Architecture

### Data Model

**Content Chunk Entity**:
- chunk_id: string (deterministic identifier)
- content: string (text content)
- source_url: string (original URL)
- chapter: string (chapter identifier)
- section: string (section identifier)
- language: string (language code)
- version: string (content version)

**Vector Record Entity**:
- vector_id: string (Qdrant point ID)
- embedding: float[] (vector representation)
- metadata: object (source_url, chapter, section, chunk_id, language, version)

### API Contracts

Not applicable for this backend script - this will be a standalone pipeline.

### Architecture

**Single File Design** (main.py):
- get_all_urls(): Discover all pages from the target website
- extract_text_from_urls(): Extract clean text from each URL
- chunk_text(): Split content into semantically coherent chunks
- embed(): Generate embeddings using Cohere
- create_collection(): Initialize Qdrant collection named 'physical_ai_humanoid_docs_v1'
- save_chunk_to_qdrant(): Store embeddings with metadata in Qdrant
- main(): Execute the complete pipeline

## Phase 2: Implementation Approach

### Dependencies
- cohere: For embedding generation
- qdrant-client: For vector storage
- requests/BeautifulSoup: For content extraction
- python-dotenv: For environment variable management

### Execution Flow
1. Initialize Cohere and Qdrant clients
2. Discover all URLs from the target site
3. Extract clean text from each URL
4. Chunk the content preserving semantic boundaries
5. Generate embeddings for each chunk
6. Store vectors in Qdrant with metadata
7. Validate successful storage

## Risk Analysis

- **Website structure changes**: Content extraction may break if site structure changes
- **Rate limiting**: Cohere and website may have rate limits
- **Large content volumes**: Processing may take significant time
- **Network failures**: External dependencies may be unavailable

## Next Steps

1. Complete Phase 0 research to resolve unknowns
2. Begin implementation following the architecture design
3. Create the main.py file with the specified functions
4. Test with the target URL