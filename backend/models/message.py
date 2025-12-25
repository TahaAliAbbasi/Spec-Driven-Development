"""
Pydantic data models for Message
"""
from pydantic import BaseModel, Field
from typing import Optional, Dict, Any
from datetime import datetime
from enum import Enum


class MessageRole(str, Enum):
    USER = "user"
    ASSISTANT = "assistant"


class MessageState(str, Enum):
    PENDING = "pending"
    COMPLETED = "completed"
    FAILED = "failed"


class Message(BaseModel):
    """
    Data model for a single message in a conversation session
    """
    role: MessageRole = Field(
        ...,
        description="Role of the message sender (user or assistant)"
    )
    content: str = Field(
        ...,
        description="The text of the message"
    )
    timestamp: datetime = Field(
        default_factory=datetime.now,
        description="When the message was created"
    )
    metadata: Optional[Dict[str, Any]] = Field(
        default=None,
        description="Additional information about the message"
    )
    state: MessageState = Field(
        default=MessageState.COMPLETED,
        description="Current state of the message"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "role": "user",
                "content": "What are the key principles of humanoid robotics?",
                "timestamp": "2025-12-24T10:00:00Z",
                "state": "completed"
            }
        }