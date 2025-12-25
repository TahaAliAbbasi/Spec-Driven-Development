"""
Pydantic data models for Retrieved Context
"""
from pydantic import BaseModel, Field
from typing import List
from datetime import datetime


class RetrievedContext(BaseModel):
    """
    Data model for retrieved context chunks
    """
    content: str = Field(
        ...,
        description="The actual content of the retrieved chunk"
    )
    source_url: str = Field(
        ...,
        description="URL where the content originated"
    )
    chapter: str = Field(
        default="",
        description="Chapter or section identifier"
    )
    section: str = Field(
        default="",
        description="Specific section identifier"
    )
    chunk_id: str = Field(
        ...,
        description="Unique identifier for this chunk"
    )
    relevance_score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Similarity score between 0.0 and 1.0"
    )
    token_count: int = Field(
        ...,
        ge=0,
        description="Number of tokens in the content"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "content": "Humanoid robots are designed to resemble and mimic human behavior...",
                "source_url": "https://physical-ai-and-humanoid-robotics-lemon.vercel.app/humanoid-design",
                "chapter": "design-principles",
                "section": "introduction",
                "chunk_id": "chunk-abc123",
                "relevance_score": 0.87,
                "token_count": 150
            }
        }