"""
Pydantic data models for Generated Response
"""
from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime
from .citation import Citation


class GeneratedResponse(BaseModel):
    """
    Data model for generated responses
    """
    response_text: str = Field(
        ...,
        description="The actual response text"
    )
    confidence_score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Confidence level of the response (0.0-1.0)"
    )
    citations: List[Citation] = Field(
        default_factory=list,
        description="References to source documents"
    )
    timestamp: datetime = Field(
        default_factory=datetime.now,
        description="When the response was generated"
    )
    session_id: Optional[str] = Field(
        None,
        description="ID of the conversation session"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "response_text": "Humanoid robotics is based on several key principles...",
                "confidence_score": 0.85,
                "citations": [],
                "session_id": "session-12345"
            }
        }