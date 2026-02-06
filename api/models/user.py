"""User model definitions."""

from datetime import datetime
from typing import Optional
from uuid import UUID

from pydantic import BaseModel, EmailStr


class UserBase(BaseModel):
    """Base user model."""

    email: EmailStr
    python_level: str
    robotics_experience: str
    math_level: str


class UserCreate(UserBase):
    """Model for creating a new user."""

    password: str


class UserUpdate(BaseModel):
    """Model for updating user profile."""

    python_level: Optional[str] = None
    robotics_experience: Optional[str] = None
    math_level: Optional[str] = None


class User(UserBase):
    """User model with all fields."""

    id: UUID
    email_verified: bool = False
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True
