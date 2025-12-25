"""
Pydantic data models for User Query
"""
from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime


class UserQuery(BaseModel):
    """
    Data model for user query input
    """
    query_text: str = Field(
        ...,
        description="The actual text of the user's query",
        min_length=1,
        max_length=1000
    )
    session_id: Optional[str] = Field(
        None,
        description="ID of the current conversation session (for multi-turn interactions)"
    )
    timestamp: datetime = Field(
        default_factory=datetime.now,
        description="When the query was submitted"
    )
    user_id: Optional[str] = Field(
        None,
        description="Identifier for the user (if authenticated)"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "query_text": "What are the key principles of humanoid robotics?",
                "session_id": "session-12345"
            }
        }