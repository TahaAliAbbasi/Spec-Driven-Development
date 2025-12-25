# Phase 2: Retrieval & Context Assembly Specification

## 1. Scope & Responsibilities

### Primary Responsibilities
- Accept user queries or selected text as input
- Convert input into vector representations for similarity search
- Retrieve relevant content chunks from the vector database
- Apply metadata-aware filtering and ranking algorithms
- Assemble clean, ordered context bundles for Phase 3 consumption
- Ensure zero hallucination by returning only content present in the vector database

### Explicit Non-Responsibilities
- Generate natural language responses
- Call any LLM for answer generation
- Implement OpenAI Agents or frontend integration
- Handle authentication or personalization logic
- Perform any form of content synthesis or summarization beyond assembly

## 2. Inputs

### User Query Input
- Free-form text query from user
- Input validation: Must be non-empty string, maximum 1000 characters
- Character encoding: UTF-8 support for international queries

### Selected Text Input
- Subset of book content selected by user
- Input validation: Must be non-empty string, maximum 5000 characters
- Scope enforcement: Retrieval must be restricted to content related to selected text

### Optional Retrieval Parameters
- top_k: Number of results to retrieve (default: 5, maximum: 20)
- chapter_filter: Restrict retrieval to specific chapters (optional)
- section_filter: Restrict retrieval to specific sections (optional)
- language_constraint: Filter by content language (default: "en")
- version_constraint: Filter by content version (default: "1.0")

## 3. Retrieval Modes

### Standard Query-Based Retrieval
- Accept user query and convert to vector
- Perform similarity search against vector database
- Return top-k most relevant chunks based on cosine similarity

### Selected-Text-Only Retrieval
- Accept selected text from book content
- Convert to vector representation
- Retrieve content semantically related to the selected text
- Enforce strict scope boundaries around selected text

### Fallback Behavior
- When selected text is empty or invalid: switch to standard query-based retrieval
- When no relevant matches found: return empty context bundle with appropriate status
- When multiple valid modes possible: prioritize selected-text-only if available

## 4. Vector Search Strategy

### Similarity Search Behavior
- Use cosine similarity for relevance scoring
- Query vector dimension must match stored vector dimensions (1024 for Cohere embed-multilingual-v3.0)
- Apply uniform search parameters across all retrieval operations

### Distance / Relevance Interpretation
- Lower cosine distance indicates higher relevance
- Minimum relevance threshold: 0.3 cosine similarity score
- Results below threshold are excluded from results

### Top-K Selection Strategy
- Retrieve exactly top_k results (default: 5)
- Apply relevance threshold filter before final selection
- Maintain stable ordering for identical inputs

## 5. Metadata Filtering

### Chapter-Aware Retrieval
- Filter results by chapter when chapter_filter parameter is provided
- Verify chapter metadata matches exactly with case-insensitive comparison
- Include results from all chapters if no filter is specified

### Section-Aware Retrieval
- Filter results by section when section_filter parameter is provided
- Verify section metadata matches exactly with case-insensitive comparison
- Include results from all sections if no filter is specified

### Language and Version Constraints
- Filter results by language when language_constraint parameter is provided
- Filter results by version when version_constraint parameter is provided
- Apply both filters simultaneously when both parameters are specified

### Deduplication Rules
- Remove chunks with identical content within retrieved results
- Prioritize chunks with higher relevance scores during deduplication
- Preserve original ordering after deduplication

## 6. Ranking & Post-Processing

### Secondary Ranking Heuristics
- Apply metadata-based ranking after similarity scoring
- Prioritize content from same chapter as selected text (for selected-text mode)
- Consider content freshness if version metadata indicates updates

### Handling Near-Duplicate Chunks
- Compare content similarity using character-based overlap
- Remove chunks with >80% content overlap
- Preserve chunk with higher relevance score

### Stable, Deterministic Ordering
- Sort results primarily by relevance score (descending)
- Use chunk_id as tiebreaker for consistent ordering
- Ensure identical inputs produce identical output ordering

## 7. Context Assembly Rules

### Maximum Context Size Constraints
- Total context size must not exceed 4000 tokens
- Calculate token count using the same heuristic as Phase 1 (words * 1.33)
- Truncate results if necessary, preserving highest-ranked items

### Ordering Rules
- Order by relevance score (descending) as primary criterion
- Preserve semantic structure when chunks originate from same source
- Maintain document order when chunks come from same URL

### Chunk Boundary Preservation
- Do not modify original chunk content during assembly
- Preserve all metadata associated with each chunk
- Maintain chunk_id references for traceability

### No Hallucinated or Synthetic Content
- Return only original content from vector database
- No content modification, summarization, or synthesis
- Preserve exact original text and metadata

## 8. Selected-Text Enforcement

### Hard Guarantees
- Retrieval must be restricted to content semantically related to selected text
- No external context leakage beyond selected text scope
- Apply strict semantic boundaries during vector search

### Zero Leakage Outside Selected Text Scope
- Implement content scope validation
- Reject results that exceed selected text semantic boundaries
- Return explicit error if scope cannot be maintained

### Explicit Rejection Behavior
- When scope violation is detected: return error status with descriptive message
- When selected text is invalid: fall back to standard query mode
- Log scope violation attempts for observability

## 9. Error & Edge Case Handling

### Empty Database
- Detect when vector database has no content
- Return appropriate error status with descriptive message
- Log condition for system monitoring

### No Relevant Matches
- Return empty context bundle with status indicating no matches
- Include query parameters in response for debugging
- Log zero-match queries for analysis

### Partial Matches
- Return all matches that meet minimum relevance threshold
- Indicate number of matches returned in response
- Include confidence indicators for each result

### Conflicting Metadata
- Detect metadata inconsistencies in retrieved chunks
- Apply validation rules to identify conflicts
- Return warning status when conflicts are detected

## 10. Observability

### Retrieval Logs
- Log every retrieval request with timestamp, query type, and parameters
- Record response time for each retrieval operation
- Log number of results returned and relevance thresholds

### Metrics
- Query count: Track total number of retrieval requests
- Hit rate: Percentage of queries returning non-empty results
- Latency: 95th percentile response time for retrieval operations
- Error rate: Percentage of queries resulting in errors

### Debug Traceability
- Include request ID for tracking individual retrieval operations
- Log chunk IDs of all retrieved results
- Record metadata filters applied during retrieval

## 11. Security & Compliance

### No Exposure of Raw Embeddings
- Never return raw vector embeddings in API responses
- Expose only content, metadata, and relevance scores
- Validate output does not contain embedding data

### No Persistence of User Queries
- Do not store user queries beyond immediate processing
- Clear query data from memory after processing
- Log only anonymized metrics, not actual query content

### Compliance with Zero-Hallucination Policy
- Return only content present in vector database
- Verify all returned content has corresponding vector storage
- Implement validation checks to prevent synthetic content generation

## 12. Deliverables

### Retrieval Module or Service
- Single cohesive module with clear public interface
- Function signature: retrieve_context(query: str, selected_text: Optional[str] = None, params: RetrievalParams) -> ContextBundle
- Support both query-based and selected-text retrieval modes

### Deterministic Output Schema
- ContextBundle containing: retrieved_chunks[], metadata, status, and metrics
- RetrievedChunk containing: content, source_url, chapter, section, chunk_id, relevance_score
- Consistent field names and data types across all responses

### Clear Handoff Artifacts for Phase 3
- Well-documented API for context retrieval
- Sample test queries and expected responses
- Integration guidelines for Phase 3 agents

## Success Criteria

1. Retrieval returns ONLY content present in the vector database with 100% accuracy
2. Selected-text-only mode never leaks external context beyond semantic boundaries
3. Results are deterministic across repeated identical queries (same order, same content)
4. Retrieved context is sufficient for answer generation in Phase 3 (comprehensive and relevant)
5. Phase 2 can be tested independently without any LLM calls or external dependencies
6. System handles all edge cases gracefully with appropriate error responses
7. Performance meets latency requirements (retrieval under 1 second p95)
8. All security and compliance requirements are satisfied