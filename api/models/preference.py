"""User preference model definitions."""

from datetime import datetime
from typing import Optional
from uuid import UUID

from pydantic import BaseModel


class UserPreference(BaseModel):

    """User preference for a chapter."""
    id: UUID
    user_id: UUID
    chapter_slug: str
    display_language: str = "en"
    content_depth: str = "default"  # beginner, default, advanced
    last_visited: datetime

    class Config:
        from_attributes = True
