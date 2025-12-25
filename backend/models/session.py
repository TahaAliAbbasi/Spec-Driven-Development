"""
Pydantic data models for Conversation Session
"""
from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime
from .message import Message


class ConversationSession(BaseModel):
    """
    Data model for conversation session
    """
    session_id: str = Field(
        ...,
        description="Unique identifier for the conversation session"
    )
    created_at: datetime = Field(
        default_factory=datetime.now,
        description="When the session started"
    )
    last_interaction: datetime = Field(
        default_factory=datetime.now,
        description="When the last message was exchanged"
    )
    messages: List[Message] = Field(
        default_factory=list,
        description="History of messages in this session"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "session_id": "session-12345",
                "created_at": "2025-12-24T10:00:00Z",
                "last_interaction": "2025-12-24T10:05:00Z",
                "messages": []
            }
        }