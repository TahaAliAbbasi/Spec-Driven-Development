# Research Findings: Embedding Pipeline Setup

**Feature**: 2-knowledge-ingestion
**Date**: 2025-12-13

## Decision 1: Cohere Embedding Model Selection

**Decision**: Use Cohere's embed-multilingual-v3.0 model
**Rationale**: This model is optimized for multilingual content and provides good performance for book content which may include technical terms. It's the latest version with strong semantic understanding capabilities.
**Alternatives considered**:
- embed-english-v3.0: Less suitable if content has mixed languages
- embed-multilingual-light-v3.0: Less powerful but faster alternative

## Decision 2: Chunking Parameters

**Decision**: Use 512 token chunks with 10% overlap (51 tokens)
**Rationale**: This provides good semantic boundaries while maintaining context. 512 tokens is sufficient for most book sections while being efficient for embedding generation.
**Alternatives considered**:
- 256 tokens: Smaller chunks with more overlap needed
- 1000 tokens: Larger chunks that might cross semantic boundaries

## Decision 3: Qdrant Collection Configuration

**Decision**: Create collection named 'physical_ai_humanoid_docs_v1' with:
- Vector size: 1024 (for multilingual-v3.0 model)
- Distance function: Cosine
- Payload schema: Contains source_url, chapter, section, chunk_id, language, version
**Rationale**: Matches Cohere's output dimensions and provides appropriate metadata for RAG retrieval
**Alternatives considered**:
- Different distance functions: Euclidean, Dot product
- Different vector sizes: Based on different models

## Additional Research Findings

### Content Extraction Strategy
- Use requests and BeautifulSoup for initial content extraction
- Focus on main content areas, excluding navigation and footer
- Preserve headings and code blocks in text extraction

### URL Discovery
- Use sitemap.xml if available
- Implement breadth-first crawling with depth limit
- Respect robots.txt and rate limiting

### Error Handling
- Implement retry mechanisms for network failures
- Use exponential backoff for API calls
- Log failures for debugging and monitoring