# Data Model: Retrieval & Context Assembly

## Core Entities

### RetrievalRequest
**Purpose**: Represents a request for context retrieval from the RAG system

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| query | string | No | null | User query text for standard retrieval |
| selected_text | string | No | null | Text selected by user for scope-restricted retrieval |
| top_k | integer | No | 5 | Number of results to retrieve (max 20) |
| chapter_filter | string | No | null | Filter results by specific chapter |
| section_filter | string | No | null | Filter results by specific section |
| language_constraint | string | No | "en" | Filter results by content language |
| version_constraint | string | No | "1.0" | Filter results by content version |

**Validation Rules**:
- Either `query` OR `selected_text` must be provided (not both null)
- `top_k` must be between 1 and 20
- `query` length must be between 1 and 1000 characters
- `selected_text` length must be between 1 and 5000 characters

### RetrievedChunk
**Purpose**: Represents a single content chunk retrieved from the vector database

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| content | string | Yes | Original content from the chunk |
| source_url | string | Yes | URL where the content originated |
| chapter | string | Yes | Chapter identifier from metadata |
| section | string | Yes | Section identifier from metadata |
| chunk_id | string | Yes | Unique identifier from Qdrant |
| relevance_score | float | Yes | Cosine similarity score (0.0-1.0) |
| token_count | integer | Yes | Number of tokens in the chunk |

**Validation Rules**:
- `relevance_score` must be between 0.0 and 1.0
- `token_count` must be positive
- All string fields must not be empty

### ContextBundle
**Purpose**: Represents the assembled context returned to the caller

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| retrieved_chunks | List[RetrievedChunk] | Yes | Ordered list of relevant chunks |
| metadata | object | Yes | Additional information about the retrieval |
| status | string | Yes | Result status ("success", "partial", "no_matches", "error") |
| total_tokens | integer | Yes | Total token count of all chunks |

**Metadata Fields**:
| Field | Type | Description |
|-------|------|-------------|
| query_type | string | Type of query ("standard", "selected_text") |
| processing_time_ms | integer | Time taken for retrieval in milliseconds |
| original_top_k | integer | Requested number of results |
| returned_count | integer | Actual number of results returned |

**Validation Rules**:
- `total_tokens` must not exceed MAX_CONTEXT_TOKENS (4000)
- `status` must be one of the allowed values
- `retrieved_chunks` length must match `returned_count` in metadata

## State Transitions

### RetrievalRequest Processing Flow
1. **Received**: Request validated and parsed
2. **Vectorized**: Query/selected_text converted to vector representation
3. **Searched**: Vector search performed against Qdrant database
4. **Filtered**: Results filtered by metadata constraints
5. **Ranked**: Results ordered by relevance and other criteria
6. **Assembled**: Context bundle created within token limits
7. **Completed**: Response returned to caller

### ContextBundle States
- **Empty**: No relevant results found (status: "no_matches")
- **Partial**: Some results found but below threshold (status: "partial")
- **Complete**: Sufficient relevant results found (status: "success")
- **Error**: Retrieval failed due to system issues (status: "error")

## Relationships

### RetrievalRequest → ContextBundle
- One request generates one context bundle
- Request parameters influence the content and structure of the bundle

### ContextBundle → RetrievedChunk
- One context bundle contains 0 to many retrieved chunks
- Chunks are ordered by relevance score and other ranking criteria

## Constraints

### Token Constraints
- Individual chunk token count: unlimited (but affects total)
- Total bundle token count: maximum 4000 tokens
- Enforced during assembly phase

### Relevance Constraints
- Minimum relevance score: 0.3 cosine similarity
- Applied during filtering phase
- Results below threshold are excluded

### Scope Constraints (Selected-Text Mode)
- Retrieved content must be semantically related to selected text
- Enforced through vector similarity validation
- Violations result in error status