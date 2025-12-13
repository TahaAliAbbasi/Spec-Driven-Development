# Data Model: Knowledge Ingestion & Vectorization

**Feature**: 2-knowledge-ingestion
**Date**: 2025-12-13

## Content Chunk Entity

**Description**: Represents a semantically coherent piece of text from the book

**Fields**:
- chunk_id: string (required) - Deterministic identifier for the chunk
- content: string (required) - The actual text content of the chunk
- source_url: string (required) - Original URL where the content was found
- chapter: string (optional) - Chapter identifier if applicable
- section: string (optional) - Section identifier if applicable
- language: string (required) - Language code (default: 'en')
- version: string (required) - Content version for tracking updates

**Validation Rules**:
- content must not be empty
- source_url must be a valid URL
- chunk_id must be unique within the system
- language must be a valid ISO language code

## Vector Record Entity

**Description**: Stores the embedding vector in Qdrant with associated metadata

**Fields**:
- vector_id: string (required) - Qdrant point ID (same as chunk_id)
- embedding: float[] (required) - 1024-dimensional vector from Cohere
- metadata: object (required) - Contains source_url, chapter, section, chunk_id, language, version

**Validation Rules**:
- embedding must have exactly 1024 dimensions
- metadata must contain all required fields
- vector_id must match the chunk_id format

## Relationships

- Each Content Chunk maps to exactly one Vector Record
- Vector Records are stored in the 'physical_ai_humanoid_docs_v1' Qdrant collection

## State Transitions

- Content Chunk: PENDING → PROCESSED → STORED
- Vector Record: CREATED → EMBEDDED → INDEXED