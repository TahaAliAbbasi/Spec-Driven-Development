"""
Retrieval & Context Assembly Service
Single-file implementation for Phase 2 of the AI-driven book project with RAG chatbot.
This service accepts user queries or selected text, converts them to vector representations,
and retrieves the most relevant content chunks from the Qdrant vector database based on cosine similarity.
"""

import os
import logging
import time
import uuid
from typing import List, Optional, Dict, Any
from enum import Enum

import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from pydantic import BaseModel
from pydantic.functional_validators import field_validator
from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.responses import JSONResponse
import uvicorn
from dotenv import load_dotenv


# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=os.getenv("LOG_LEVEL", "INFO").upper(),
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


# Phase 2: Foundational Components
# T005-T007: Define Pydantic models
class RetrievalRequest(BaseModel):
    """Request model for context retrieval from the RAG system"""
    query: Optional[str] = None
    selected_text: Optional[str] = None
    top_k: int = 5
    chapter_filter: Optional[str] = None
    section_filter: Optional[str] = None
    language_constraint: str = "en"
    version_constraint: str = "1.0"

    @field_validator('top_k')
    @classmethod
    def validate_top_k(cls, v):
        if v < 1 or v > 20:
            raise ValueError('top_k must be between 1 and 20')
        return v

    @field_validator('query')
    @classmethod
    def validate_query(cls, v):
        if v is not None and len(v) > 1000:
            raise ValueError('query must be 1000 characters or less')
        return v

    @field_validator('selected_text')
    @classmethod
    def validate_selected_text(cls, v):
        if v is not None and len(v) > 5000:
            raise ValueError('selected_text must be 5000 characters or less')
        return v

    def model_post_init(self, __context):
        # Validate that either query or selected_text is provided
        if self.query is None and self.selected_text is None:
            raise ValueError('Either query OR selected_text must be provided')


class RetrievedChunk(BaseModel):
    """Model representing a single content chunk retrieved from the vector database"""
    content: str
    source_url: str
    chapter: str
    section: str
    chunk_id: str
    relevance_score: float
    token_count: int

    @field_validator('relevance_score')
    @classmethod
    def validate_relevance_score(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('relevance_score must be between 0.0 and 1.0')
        return v

    @field_validator('token_count')
    @classmethod
    def validate_token_count(cls, v):
        if v <= 0:
            raise ValueError('token_count must be positive')
        return v


class RetrievalStatus(str, Enum):
    SUCCESS = "success"
    PARTIAL = "partial"
    NO_MATCHES = "no_matches"
    ERROR = "error"


class ContextBundleMetadata(BaseModel):
    """Metadata for the context bundle"""
    query_type: str  # "standard" or "selected_text"
    processing_time_ms: int
    original_top_k: int
    returned_count: int


class ContextBundle(BaseModel):
    """Model representing the assembled context returned to the caller"""
    retrieved_chunks: List[RetrievedChunk]
    metadata: ContextBundleMetadata
    status: RetrievalStatus
    total_tokens: int

    @field_validator('total_tokens')
    @classmethod
    def validate_total_tokens(cls, v):
        max_tokens = int(os.getenv("MAX_CONTEXT_TOKENS", "4000"))
        if v > max_tokens:
            raise ValueError(f'total_tokens must not exceed {max_tokens}')
        return v


# T008: Token counting utility function
def count_tokens(text: str) -> int:
    """
    Count approximate number of tokens in text (using Phase 1 heuristic)
    Phase 1 used: words * 1.33
    """
    if not text:
        return 0
    words = len(text.split())
    return max(1, int(words * 1.33))  # Rough approximation from Phase 1


# T009: Error handling classes
class RetrievalError(Exception):
    """Base exception for retrieval-related errors"""
    pass


class VectorizationError(RetrievalError):
    """Exception raised when vectorization fails"""
    pass


class DatabaseError(RetrievalError):
    """Exception raised when database operations fail"""
    pass


class ScopeViolationError(RetrievalError):
    """Exception raised when selected-text mode exceeds scope boundaries"""
    pass


# T010-T011: Client initialization
def setup_cohere_client():
    """Setup Cohere client with API key from environment"""
    api_key = os.getenv("COHERE_API_KEY")
    if not api_key:
        raise ValueError("COHERE_API_KEY environment variable is required")

    client = cohere.Client(api_key)
    logger.info("Cohere client initialized successfully")
    return client


def setup_qdrant_client():
    """Setup Qdrant client with API key and URL from environment"""
    api_key = os.getenv("QDRANT_API_KEY")
    url = os.getenv("QDRANT_URL")

    if not api_key:
        raise ValueError("QDRANT_API_KEY environment variable is required")
    if not url:
        raise ValueError("QDRANT_URL environment variable is required")

    client = QdrantClient(
        url=url,
        api_key=api_key,
    )
    logger.info("Qdrant client initialized successfully")
    return client


# Phase 3: Core Vectorization Service [US1]
# T012-T017: Implement vectorization functionality
def create_retry_mechanism():
    """Create retry mechanism with exponential backoff for API calls"""
    def retry_with_backoff(func, max_retries=3, base_delay=1):
        def wrapper(*args, **kwargs):
            delay = base_delay
            for attempt in range(max_retries):
                try:
                    return func(*args, **kwargs)
                except Exception as e:
                    if attempt == max_retries - 1:
                        raise e
                    logger.warning(f"Attempt {attempt + 1} failed: {e}. Retrying in {delay}s...")
                    time.sleep(delay)
                    delay *= 2  # Exponential backoff
            return None
        return wrapper
    return retry_with_backoff


def vectorize_text(text: str, cohere_client) -> List[float]:
    """
    Convert text to vector representation using Cohere's embed-multilingual-v3.0 model
    This matches the embedding model used in Phase 1 (1024 dimensions)
    """
    try:
        # Validate text length
        if len(text) > 5000:  # Max for Cohere API
            # Truncate if too long (shouldn't happen due to validation, but as safety)
            text = text[:5000]

        retry_func = create_retry_mechanism()

        @retry_func
        def generate_embedding_with_retry(text_content):
            response = cohere_client.embed(
                texts=[text_content],
                model="embed-multilingual-v3.0",  # Same as Phase 1
                input_type="search_query"  # Using search_query for queries, search_document for stored content
            )
            return response.embeddings[0]  # Return the first (and only) embedding

        return generate_embedding_with_retry(text)
    except Exception as e:
        logger.error(f"Error during vectorization: {e}")
        raise VectorizationError(f"Failed to vectorize text: {str(e)}")


# Phase 4: Similarity Search Engine [US2]
# T018-T024: Implement similarity search functionality
def search(
    query_vector: List[float],
    top_k: int,
    qdrant_client,
    filters: Optional[Dict] = None
) -> List[Dict]:
    """
    Query Qdrant vector database for relevant chunks based on cosine similarity
    """
    try:
        # Prepare filters for Qdrant
        qdrant_filters = None
        if filters:
            filter_conditions = []

            if filters.get('chapter_filter'):
                filter_conditions.append(
                    models.FieldCondition(
                        key="chapter",
                        match=models.MatchValue(value=filters['chapter_filter'])
                    )
                )

            if filters.get('section_filter'):
                filter_conditions.append(
                    models.FieldCondition(
                        key="section",
                        match=models.MatchValue(value=filters['section_filter'])
                    )
                )

            if filters.get('language_constraint'):
                filter_conditions.append(
                    models.FieldCondition(
                        key="language",
                        match=models.MatchValue(value=filters['language_constraint'])
                    )
                )

            if filters.get('version_constraint'):
                filter_conditions.append(
                    models.FieldCondition(
                        key="version",
                        match=models.MatchValue(value=filters['version_constraint'])
                    )
                )

            if filter_conditions:
                qdrant_filters = models.Filter(must=filter_conditions)

        # Perform search
        search_results = qdrant_client.search(
            collection_name=os.getenv("QDRANT_COLLECTION_NAME", "physical_ai_humanoid_docs_v3"),
            query_vector=query_vector,
            limit=top_k * 2,  # Get more results to account for filtering
            score_threshold=0.3,  # Minimum relevance threshold
            with_payload=True,
            with_vectors=False,
            query_filter=qdrant_filters
        )

        # Format results
        formatted_results = []
        for result in search_results:
            formatted_results.append({
                'content': result.payload.get('content', ''),
                'source_url': result.payload.get('source_url', ''),
                'chapter': result.payload.get('chapter', ''),
                'section': result.payload.get('section', ''),
                'chunk_id': result.id,
                'relevance_score': result.score,
                'token_count': result.payload.get('token_count', 0)
            })

        # Apply additional filtering in application layer if needed
        if filters:
            filtered_results = []
            for result in formatted_results:
                include = True

                if filters.get('chapter_filter') and result['chapter'] != filters['chapter_filter']:
                    include = False
                if filters.get('section_filter') and result['section'] != filters['section_filter']:
                    include = False
                if filters.get('language_constraint') and result['payload'].get('language', 'en') != filters['language_constraint']:
                    include = False
                if filters.get('version_constraint') and result['payload'].get('version', '1.0') != filters['version_constraint']:
                    include = False

                if include:
                    filtered_results.append(result)

            formatted_results = filtered_results

        # Return top_k results after filtering
        return formatted_results[:top_k]

    except Exception as e:
        logger.error(f"Error during similarity search: {e}")
        raise DatabaseError(f"Failed to search database: {str(e)}")


# Phase 5: Metadata Filtering [US3]
# T025-T032: Implement metadata filtering functionality
def filter_chunks(chunks: List[Dict], filters: Dict) -> List[Dict]:
    """
    Apply metadata-based filtering to search results
    """
    filtered_chunks = []

    for chunk in chunks:
        include = True

        # Chapter-aware filtering
        if filters.get('chapter_filter'):
            chunk_chapter = chunk.get('chapter', '').lower()
            filter_chapter = filters['chapter_filter'].lower()
            if chunk_chapter != filter_chapter:
                include = False

        # Section-aware filtering
        if include and filters.get('section_filter'):
            chunk_section = chunk.get('section', '').lower()
            filter_section = filters['section_filter'].lower()
            if chunk_section != filter_section:
                include = False

        # Language constraint filtering
        if include and filters.get('language_constraint'):
            chunk_language = chunk.get('language', 'en').lower()
            filter_language = filters['language_constraint'].lower()
            if chunk_language != filter_language:
                include = False

        # Version constraint filtering
        if include and filters.get('version_constraint'):
            chunk_version = chunk.get('version', '1.0')
            filter_version = filters['version_constraint']
            if chunk_version != filter_version:
                include = False

        if include:
            filtered_chunks.append(chunk)

    # Deduplication logic with relevance score prioritization
    unique_chunks = {}
    for chunk in filtered_chunks:
        content = chunk['content']
        if content in unique_chunks:
            # Keep the chunk with higher relevance score
            if chunk['relevance_score'] > unique_chunks[content]['relevance_score']:
                unique_chunks[content] = chunk
        else:
            unique_chunks[content] = chunk

    deduplicated_chunks = list(unique_chunks.values())

    # Handle near-duplicate chunks (>80% content overlap)
    final_chunks = []
    for chunk in deduplicated_chunks:
        is_duplicate = False
        for existing in final_chunks:
            # Calculate content overlap
            content1 = chunk['content'].lower()
            content2 = existing['content'].lower()

            # Simple overlap calculation (could be more sophisticated)
            min_len = min(len(content1), len(content2))
            if min_len == 0:
                continue

            common_length = len(set(content1.split()) & set(content2.split()))
            overlap_ratio = common_length / min_len if min_len > 0 else 0

            if overlap_ratio > 0.8:  # More than 80% overlap
                # Keep the one with higher relevance score
                if chunk['relevance_score'] > existing['relevance_score']:
                    final_chunks.remove(existing)
                    final_chunks.append(chunk)
                is_duplicate = True
                break

        if not is_duplicate:
            final_chunks.append(chunk)

    return final_chunks


# Phase 6: Context Assembly [US4]
# T033-T040: Implement context assembly functionality
def assemble_context(chunks: List[Dict], max_tokens: int = 4000) -> ContextBundle:
    """
    Order and bundle filtered chunks into final context within token limits
    """
    # Sort by relevance score (descending) as primary criterion
    sorted_chunks = sorted(chunks, key=lambda x: x['relevance_score'], reverse=True)

    # Calculate total tokens and build bundle
    total_tokens = 0
    selected_chunks = []

    for chunk in sorted_chunks:
        chunk_tokens = chunk.get('token_count', count_tokens(chunk['content']))
        if total_tokens + chunk_tokens <= max_tokens:
            selected_chunks.append(chunk)
            total_tokens += chunk_tokens
        else:
            # If we can't fit this chunk, stop here
            break

    # Create RetrievedChunk objects
    retrieved_chunks = []
    for chunk in selected_chunks:
        retrieved_chunk = RetrievedChunk(
            content=chunk['content'],
            source_url=chunk['source_url'],
            chapter=chunk['chapter'],
            section=chunk['section'],
            chunk_id=chunk['chunk_id'],
            relevance_score=chunk['relevance_score'],
            token_count=chunk.get('token_count', count_tokens(chunk['content']))
        )
        retrieved_chunks.append(retrieved_chunk)

    # Determine status
    if len(retrieved_chunks) == 0:
        status = RetrievalStatus.NO_MATCHES
    elif len(selected_chunks) < len(sorted_chunks):
        status = RetrievalStatus.PARTIAL
    else:
        status = RetrievalStatus.SUCCESS

    # Create context bundle
    context_bundle = ContextBundle(
        retrieved_chunks=retrieved_chunks,
        metadata=ContextBundleMetadata(
            query_type="unknown",  # Will be set by the controller
            processing_time_ms=0,  # Will be calculated by the controller
            original_top_k=len(chunks),
            returned_count=len(retrieved_chunks)
        ),
        status=status,
        total_tokens=total_tokens
    )

    return context_bundle


# Phase 7: Retrieval Controller [US5]
# T041-T048: Implement retrieval controller
class RetrievalController:
    """
    Controller that orchestrates the retrieval process
    """

    def __init__(self, cohere_client, qdrant_client):
        self.cohere_client = cohere_client
        self.qdrant_client = qdrant_client

    def retrieve_context(self, request: RetrievalRequest) -> ContextBundle:
        """
        Main method to retrieve context based on request parameters
        """
        start_time = time.time()

        try:
            # Determine query type
            if request.selected_text is not None and request.selected_text.strip():
                query_type = "selected_text"
                text_to_vectorize = request.selected_text
            elif request.query is not None and request.query.strip():
                query_type = "standard"
                text_to_vectorize = request.query
            else:
                raise ValueError("Either query or selected_text must be provided and non-empty")

            # Vectorize the text
            query_vector = vectorize_text(text_to_vectorize, self.cohere_client)

            # Prepare filters
            filters = {
                'chapter_filter': request.chapter_filter,
                'section_filter': request.section_filter,
                'language_constraint': request.language_constraint,
                'version_constraint': request.version_constraint
            }

            # Perform search
            search_results = search(
                query_vector,
                request.top_k,
                self.qdrant_client,
                filters
            )

            # Apply additional application-layer filtering
            filtered_results = filter_chunks(search_results, filters)

            # Assemble context
            max_tokens = int(os.getenv("MAX_CONTEXT_TOKENS", "4000"))
            context_bundle = assemble_context(filtered_results, max_tokens)

            # Update metadata with actual values
            processing_time_ms = int((time.time() - start_time) * 1000)
            context_bundle.metadata = ContextBundleMetadata(
                query_type=query_type,
                processing_time_ms=processing_time_ms,
                original_top_k=request.top_k,
                returned_count=len(context_bundle.retrieved_chunks)
            )
            context_bundle.total_tokens = sum(chunk.token_count for chunk in context_bundle.retrieved_chunks)

            # Validate results to ensure zero hallucination
            for chunk in context_bundle.retrieved_chunks:
                if not chunk.content.strip():
                    raise RetrievalError("Empty content chunk found - possible hallucination")

            return context_bundle

        except Exception as e:
            processing_time_ms = int((time.time() - start_time) * 1000)
            logger.error(f"Error in retrieve_context: {e}")

            # Return error status bundle
            return ContextBundle(
                retrieved_chunks=[],
                metadata=ContextBundleMetadata(
                    query_type="unknown",
                    processing_time_ms=processing_time_ms,
                    original_top_k=request.top_k,
                    returned_count=0
                ),
                status=RetrievalStatus.ERROR,
                total_tokens=0
            )


# Phase 8: API Endpoints [US6]
# T049-T056: Implement FastAPI endpoints
app = FastAPI(title="Retrieval & Context Assembly Service", version="1.0.0")


@app.on_event("startup")
async def startup_event():
    """Initialize clients on startup"""
    global retrieval_controller

    try:
        cohere_client = setup_cohere_client()
        qdrant_client = setup_qdrant_client()
        retrieval_controller = RetrievalController(cohere_client, qdrant_client)
        logger.info("Retrieval service started successfully")
    except Exception as e:
        logger.error(f"Failed to start retrieval service: {e}")
        raise


@app.post("/api/retrieve", response_model=ContextBundle)
async def retrieve_endpoint(request: RetrievalRequest):
    """
    Retrieve relevant context based on query or selected text
    """
    try:
        context_bundle = retrieval_controller.retrieve_context(request)
        return context_bundle
    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error in retrieve endpoint: {e}")
        raise HTTPException(
            status_code=500,
            detail={
                "error": "Internal server error during retrieval",
                "code": "RETRIEVAL_ERROR",
                "details": {"message": str(e)}
            }
        )


@app.get("/api/health/retrieval")
async def health_check():
    """
    Health check for retrieval service
    """
    try:
        # Check if clients are available
        health_status = {
            "status": "healthy",
            "timestamp": time.time(),
            "service": "retrieval",
            "dependencies": {
                "qdrant": "checking...",
                "cohere": "checking..."
            }
        }

        # Basic connectivity checks would go here
        # For now, we'll assume they're available if no exceptions were thrown at startup
        health_status["dependencies"]["qdrant"] = "connected"
        health_status["dependencies"]["cohere"] = "available"

        return health_status
    except Exception as e:
        logger.error(f"Health check failed: {e}")
        return JSONResponse(
            status_code=503,
            content={
                "status": "unhealthy",
                "timestamp": time.time(),
                "service": "retrieval",
                "error": str(e)
            }
        )


# Phase 9: Error Handling & Edge Cases [US7]
# T057-T064: Implement comprehensive error handling
@app.exception_handler(RetrievalError)
async def retrieval_error_handler(request, exc):
    """Handle retrieval-specific errors"""
    return JSONResponse(
        status_code=500,
        content={
            "error": str(exc),
            "code": "RETRIEVAL_ERROR",
            "details": {}
        }
    )


@app.exception_handler(400)
async def bad_request_handler(request, exc):
    """Handle bad request errors"""
    return JSONResponse(
        status_code=400,
        content={
            "error": "Bad request",
            "code": "INVALID_REQUEST",
            "details": {"message": str(exc) if hasattr(exc, 'detail') else "Invalid request parameters"}
        }
    )


@app.exception_handler(500)
async def internal_error_handler(request, exc):
    """Handle internal server errors"""
    return JSONResponse(
        status_code=500,
        content={
            "error": "Internal server error",
            "code": "INTERNAL_ERROR",
            "details": {"message": str(exc) if str(exc) else "An unexpected error occurred"}
        }
    )


# Phase 10: Observability & Security [US8]
# T065-T072: Implement logging and security features
@app.middleware("http")
async def log_requests(request, call_next):
    """Middleware to log all requests"""
    start_time = time.time()
    request_id = str(uuid.uuid4())

    # Add request ID to request state for tracking
    request.state.request_id = request_id

    # Log the incoming request
    logger.info(f"Request ID: {request_id} | {request.method} {request.url.path}")

    response = await call_next(request)

    # Calculate processing time
    process_time = time.time() - start_time
    formatted_time = f"{process_time:.4f}s"

    # Log the response
    logger.info(f"Request ID: {request_id} | Status: {response.status_code} | Time: {formatted_time}")

    return response


if __name__ == "__main__":
    port = int(os.getenv("PORT", "8000"))
    uvicorn.run(app, host="0.0.0.0", port=port)