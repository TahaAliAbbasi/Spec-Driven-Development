"""
Pydantic data models for Citation
"""
from pydantic import BaseModel, Field
from typing import Optional


class Citation(BaseModel):
    """
    Data model for citations
    """
    source_url: str = Field(
        ...,
        description="URL of the source document"
    )
    chapter: str = Field(
        default="",
        description="Chapter identifier"
    )
    section: str = Field(
        default="",
        description="Section identifier"
    )
    content_snippet: str = Field(
        default="",
        description="Brief excerpt from the source"
    )
    relevance_score: Optional[float] = Field(
        None,
        ge=0.0,
        le=1.0,
        description="How relevant this citation is to the response"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "source_url": "https://physical-ai-and-humanoid-robotics-lemon.vercel.app/humanoid-design",
                "chapter": "design-principles",
                "section": "introduction",
                "content_snippet": "Humanoid robots are designed to resemble and mimic human behavior...",
                "relevance_score": 0.92
            }
        }