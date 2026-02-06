"""Chat model definitions."""

from datetime import datetime
from typing import List, Optional
from uuid import UUID

from pydantic import BaseModel


class ChatSession(BaseModel):
    """Chat session model."""

    id: UUID
    user_id: Optional[UUID] = None
    session_token: str
    created_at: datetime
    last_activity: datetime
    is_active: bool = True

    class Config:
        from_attributes = True


class ChatMessage(BaseModel):
    """Chat message model."""

    id: UUID
    session_id: UUID
    role: str  # user, assistant, system
    content: str
    language: str = "en"
    selected_text: Optional[str] = None
    chapter_context: Optional[str] = None
    sources: Optional[List[dict]] = None
    tokens_used: Optional[int] = None
    created_at: datetime

    class Config:
        from_attributes = True
