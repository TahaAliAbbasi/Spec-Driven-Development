# Research Summary: Retrieval & Context Assembly

## Vectorization Technology Research

**Decision**: Use Cohere's embed-multilingual-v3.0 model for query vectorization
**Rationale**: This matches the embedding model used in Phase 1, ensuring compatibility with the existing vector database. The model produces 1024-dimensional vectors with cosine similarity, which aligns with our Qdrant collection setup.
**Alternatives considered**:
- OpenAI embeddings: Different dimensions (1536) would require separate collection
- Sentence-transformers: Would require local model hosting and maintenance
- Hugging Face models: Similar infrastructure requirements as sentence-transformers

## Database Query Strategy Research

**Decision**: Implement metadata filtering at application layer
**Rationale**: Provides maximum flexibility for complex filtering logic while maintaining compatibility with Qdrant's query interface. Allows for custom ranking algorithms beyond basic similarity scoring.
**Alternatives considered**:
- Qdrant payload filtering: More efficient but less flexible for complex conditions
- Pre-filtered collections: Would require maintaining multiple collections for different filters

## API Framework Research

**Decision**: Use FastAPI for the retrieval service API
**Rationale**: FastAPI provides automatic OpenAPI documentation, async support, type validation, and excellent performance. It integrates well with the existing Python ecosystem.
**Alternatives considered**:
- Flask: Simpler but lacks automatic documentation and type validation
- Django REST Framework: More complex than needed for this service
- Plain HTTP server: Would require implementing many features manually

## Context Assembly Strategy Research

**Decision**: Implement token-aware context assembly with relevance-based ordering
**Rationale**: Ensures context stays within token limits while preserving the most relevant information. Uses the same token counting method as Phase 1 for consistency.
**Alternatives considered**:
- Character-based limits: Less accurate for AI context windows
- Fixed number of chunks: Doesn't account for varying chunk sizes
- Content-aware assembly: More complex and potentially slower

## Selected-Text Enforcement Research

**Decision**: Implement semantic boundary validation using vector similarity thresholds
**Rationale**: Provides a measurable way to ensure selected-text-only mode doesn't leak external context. Can be validated and tested quantitatively.
**Alternatives considered**:
- Exact text matching: Too restrictive for semantic similarity
- Chapter/section restriction: Too broad for precise scope control
- Manual validation: Not scalable or deterministic