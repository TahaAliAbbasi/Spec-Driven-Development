# Implementation Plan: Phase 2 - Retrieval & Context Assembly

## Technical Context

The Retrieval & Context Assembly system will implement a vector search-based retrieval mechanism that works with the Qdrant vector database populated during Phase 1. The system will accept user queries or selected text, convert them to vector representations, and retrieve the most relevant content chunks based on cosine similarity.

**Architecture**: Single-file Python implementation with clear separation of concerns
**Technology Stack**: Python 3.9+, Cohere API for vectorization, Qdrant client, FastAPI for API layer
**Integration Points**:
- Input: User queries/selected text from frontend
- Output: Context bundles for Phase 3 (response generation)
- Vector Database: Qdrant Cloud with pre-populated content from Phase 1

**Key Components**:
- Vectorization service (convert text to vectors using Cohere)
- Similarity search engine (query Qdrant with vector)
- Metadata filtering layer (apply chapter/section/language filters)
- Context assembly engine (order and bundle results)
- API endpoints (expose functionality to frontend)

## Constitution Check

**Compliance Verification**: All implementation must comply with Global Constitution v2.0.0

- ✅ **Zero Hallucination Policy (Section II)**: Retrieval will ONLY return content present in the vector database, with no content modification or synthesis
- ✅ **Deterministic, Explainable AI Behavior (Section III)**: Results will be ordered deterministically with clear attribution to source chunks
- ✅ **RAG Rules (Sections IX-XIII)**: Proper chunking strategy maintained, embedding consistency ensured, context filtering implemented
- ✅ **Selected-Text-Only Answering (Section XII)**: Hard guarantees for scope restriction when selected text is provided
- ✅ **Mandatory Tech Stack Usage (Section XIX)**: Using Qdrant Cloud as required

**Gates**:
- [X] No violations identified
- [X] All constitutional requirements addressed
- [X] Implementation supports deterministic behavior

## Phase 0: Research & Discovery

### Research Findings

**Decision**: Use Cohere's embed-multilingual-v3.0 model for query vectorization to match Phase 1 embeddings
**Rationale**: Ensures compatibility with existing vector database (1024 dimensions with cosine similarity)
**Alternatives considered**: OpenAI embeddings (different dimensions), sentence-transformers (local but different model)

**Decision**: Implement metadata filtering at application layer rather than Qdrant filtering
**Rationale**: Provides more flexible filtering options and complex query logic
**Alternatives considered**: Qdrant's payload filtering (more efficient but less flexible)

**Decision**: Use FastAPI for API layer to provide clean interface to frontend
**Rationale**: FastAPI provides automatic API documentation, async support, and easy integration
**Alternatives considered**: Flask (simpler but less features), plain HTTP server (too basic)

## Phase 1: Data Model & API Contracts

### Data Model

**RetrievalRequest**
- query: str (user query text)
- selected_text: Optional[str] (text selected by user)
- top_k: int (default: 5, max: 20)
- chapter_filter: Optional[str] (chapter to filter by)
- section_filter: Optional[str] (section to filter by)
- language_constraint: str (default: "en")
- version_constraint: str (default: "1.0")

**RetrievedChunk**
- content: str (original chunk content)
- source_url: str (URL where content originated)
- chapter: str (chapter identifier)
- section: str (section identifier)
- chunk_id: str (unique chunk identifier from Qdrant)
- relevance_score: float (cosine similarity score)
- token_count: int (number of tokens in chunk)

**ContextBundle**
- retrieved_chunks: List[RetrievedChunk] (ordered list of relevant chunks)
- metadata: Dict (retrieval parameters, timing info)
- status: str ("success", "partial", "no_matches", "error")
- total_tokens: int (total tokens in context bundle)

### API Contracts

**POST /api/retrieve**
- Description: Retrieve relevant context based on query or selected text
- Request: RetrievalRequest object
- Response: ContextBundle object
- Status Codes: 200 (success), 400 (invalid request), 500 (retrieval error)

**GET /api/health/retrieval**
- Description: Health check for retrieval service
- Response: Health status object
- Status Codes: 200 (healthy), 503 (unhealthy)

## Phase 2: System Architecture

### Component Design

**VectorizationService**
- Purpose: Convert text (query or selected text) to vector representation
- Interface: `vectorize_text(text: str) -> List[float]`
- Dependencies: Cohere API client

**SimilaritySearchEngine**
- Purpose: Query Qdrant vector database for relevant chunks
- Interface: `search(query_vector: List[float], top_k: int) -> List[Dict]`
- Dependencies: Qdrant client

**MetadataFilter**
- Purpose: Apply metadata-based filtering to search results
- Interface: `filter_chunks(chunks: List[Dict], filters: Dict) -> List[Dict]`
- Dependencies: None

**ContextAssembler**
- Purpose: Order and bundle filtered chunks into final context
- Interface: `assemble_context(chunks: List[Dict], max_tokens: int) -> ContextBundle`
- Dependencies: Token counting utility

**RetrievalController**
- Purpose: Orchestrate the retrieval process and handle API requests
- Interface: `retrieve_context(request: RetrievalRequest) -> ContextBundle`
- Dependencies: All above services

### Integration Architecture

```
Frontend → FastAPI → RetrievalController → VectorizationService
                    → SimilaritySearchEngine → Qdrant Cloud
                    → MetadataFilter
                    → ContextAssembler
```

## Phase 3: Implementation Approach

### File Structure
- `backend/retrieval/main.py`: Main application with FastAPI setup
- `backend/retrieval/services/vectorization.py`: Vectorization service
- `backend/retrieval/services/search.py`: Similarity search engine
- `backend/retrieval/services/filtering.py`: Metadata filtering
- `backend/retrieval/services/assembly.py`: Context assembly
- `backend/retrieval/models/schemas.py`: Data models
- `backend/retrieval/config/settings.py`: Configuration

### Development Order
1. Set up FastAPI application structure
2. Implement data models and schemas
3. Create vectorization service
4. Implement similarity search engine
5. Build metadata filtering component
6. Develop context assembler
7. Create retrieval controller
8. Add API endpoints
9. Implement error handling and logging
10. Add health checks and monitoring

## Phase 4: Quality Assurance

### Testing Strategy
- Unit tests for each service component
- Integration tests for end-to-end retrieval
- Performance tests for response times
- Edge case testing for error conditions

### Validation Criteria
- Results are deterministic for identical inputs
- Selected-text mode respects scope boundaries
- Context size stays within token limits
- All constitutional requirements are met

## Phase 5: Deployment & Operations

### Environment Configuration
- COHERE_API_KEY: Cohere API key
- QDRANT_URL: Qdrant Cloud endpoint
- QDRANT_API_KEY: Qdrant Cloud API key
- MAX_CONTEXT_TOKENS: Maximum tokens in context bundle (default: 4000)

### Monitoring
- Retrieval success/failure rates
- Average response time
- Query volume metrics
- Error logging and alerting

## Risk Analysis

### High-Risk Items
1. Vector dimension compatibility between query and stored vectors
2. Performance degradation with large vector databases
3. Scope violation in selected-text-only mode

### Mitigation Strategies
1. Thorough testing of vectorization consistency
2. Performance testing with expected data volumes
3. Strict validation of selected-text scope boundaries