from pydantic import BaseModel, Field, field_validator
from typing import List, Optional, Dict, Any
from enum import Enum


class StatusEnum(str, Enum):
    """Enumeration for response status values"""
    ANSWERED = "answered"
    INSUFFICIENT_CONTEXT = "insufficient_context"
    REFUSED = "refused"


class ChunkReference(BaseModel):
    """Represents a citation to a specific chunk in the knowledge base."""

    chunk_id: str = Field(
        ...,
        description="Unique identifier of the referenced chunk",
        min_length=1
    )
    source_url: str = Field(
        ...,
        description="URL or identifier of the source document",
        min_length=1
    )

    @field_validator('chunk_id')
    def validate_chunk_id(cls, v):
        if not v or not v.strip():
            raise ValueError('chunk_id must not be empty')
        return v.strip()

    @field_validator('source_url')
    def validate_source_url(cls, v):
        if not v or not v.strip():
            raise ValueError('source_url must not be empty')
        return v.strip()


class AgentInput(BaseModel):
    """Represents the input to the RAG agent from the API layer."""

    query: str = Field(
        ...,
        description="The user's question or query string",
        min_length=1
    )
    context_bundle: Dict[str, Any] = Field(
        ...,
        description="The context bundle object from Phase 2 retrieval"
    )
    mode: str = Field(
        ...,
        description="Either 'global' or 'selected_text_only' to determine response behavior",
        pattern=r"^(global|selected_text_only)$"
    )

    @field_validator('query')
    def validate_query(cls, v):
        if not v or not v.strip():
            raise ValueError('Query must not be empty')
        return v.strip()

    @field_validator('context_bundle')
    def validate_context_bundle(cls, v):
        if not v:
            raise ValueError('ContextBundle must not be empty')
        # Ensure it has required fields from Phase 2
        if 'chunks' not in v or not isinstance(v['chunks'], list):
            raise ValueError('ContextBundle must contain a chunks list')
        return v

    @field_validator('mode')
    def validate_mode(cls, v):
        if v not in ['global', 'selected_text_only']:
            raise ValueError('Mode must be either "global" or "selected_text_only"')
        return v


class AgentOutput(BaseModel):
    """Represents the output from the RAG agent to the API layer."""

    answer: Optional[str] = Field(
        None,
        description="The generated answer text"
    )
    citations: List[ChunkReference] = Field(
        default_factory=list,
        description="List of source citations for the answer"
    )
    used_chunks: List[str] = Field(
        default_factory=list,
        description="List of chunk IDs that were used to generate the answer"
    )
    status: StatusEnum = Field(
        ...,
        description="Status of the response generation"
    )
    warnings: Optional[List[str]] = Field(
        default=None,
        description="Optional list of warnings about the response"
    )

    @field_validator('answer')
    def validate_answer(cls, v, values):
        # If status is "answered", answer must be present and not empty
        if 'status' in values.data and values.data['status'] == StatusEnum.ANSWERED:
            if not v or not v.strip():
                raise ValueError('Answer must be present and not empty when status is "answered"')
        return v

    @field_validator('citations')
    def validate_citations(cls, v, values):
        # If citations are present, they must correspond to actual chunks in used_chunks
        if 'used_chunks' in values.data and values.data['used_chunks']:
            citation_chunk_ids = [c.chunk_id for c in v]
            for chunk_id in citation_chunk_ids:
                if chunk_id not in values.data['used_chunks']:
                    raise ValueError(f'chunk_id "{chunk_id}" in citations must be present in used_chunks')
        return v

    @field_validator('status')
    def validate_status(cls, v, values):
        if v == StatusEnum.ANSWERED and ('answer' not in values.data or not values.data['answer']):
            raise ValueError('Status cannot be "answered" without a valid answer')
        return v