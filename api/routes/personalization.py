"""Personalization API routes."""

from typing import List, Optional
from uuid import UUID, uuid4

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from services.personalization_service import (
    ExpertiseLevel,
    UserBackground,
    get_personalization_service,
)

router = APIRouter(prefix="/personalization")


class UserPreference(BaseModel):
    """User chapter preference."""
    id: UUID

    id: UUID
    chapter_slug: str
    display_language: str = "en"
    content_depth: str = "default"
    last_visited: Optional[str] = None


class SetPreferenceRequest(BaseModel):
    """Request to set chapter preference."""

    display_language: Optional[str] = None
    content_depth: Optional[str] = None


class UserBackgroundRequest(BaseModel):
    """User background for personalization."""

    expertise_level: str = "beginner"  # beginner, intermediate, advanced
    programming_experience: bool = False
    robotics_experience: bool = False
    math_comfort: str = "basic"  # basic, intermediate, advanced
    learning_goals: List[str] = []


class PersonalizeRequest(BaseModel):
    """Request to personalize content."""

    content: str
    chapter_slug: str
    background: UserBackgroundRequest


class PersonalizeResponse(BaseModel):
    """Personalized content response."""

    content: str
    depth_level: str
    chapter_slug: str


class RecommendationResponse(BaseModel):
    """Chapter recommendation."""

    chapter: str
    reason: str


# In-memory storage (replace with database in production)
user_preferences: dict = {}


@router.post("/content", response_model=PersonalizeResponse)
async def personalize_content(request: PersonalizeRequest):
    """
    Personalize chapter content based on user background.

    Adapts the content depth, explanations, and examples
    to match the user's expertise level.
    """
    personalization_service = get_personalization_service()

    try:
        # Convert request to UserBackground
        background = UserBackground(
            expertise_level=ExpertiseLevel(request.background.expertise_level),
            programming_experience=request.background.programming_experience,
            robotics_experience=request.background.robotics_experience,
            math_comfort=request.background.math_comfort,
            learning_goals=request.background.learning_goals,
        )

        # Get personalized content
        personalized = await personalization_service.personalize_content(
            content=request.content,
            background=background,
            chapter_slug=request.chapter_slug,
        )

        depth = personalization_service.determine_content_depth(background)

        return PersonalizeResponse(
            content=personalized,
            depth_level=depth,
            chapter_slug=request.chapter_slug,
        )

    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Personalization failed: {str(e)}")

@router.post("/recommendations", response_model=List[RecommendationResponse])
async def get_recommendations(
    background: UserBackgroundRequest,
    current_chapter: str = "",
):
    """
    Get personalized chapter recommendations.

    Returns suggested chapters based on user's background
    and learning goals.
    """
    personalization_service = get_personalization_service()

    try:
        user_background = UserBackground(
            expertise_level=ExpertiseLevel(background.expertise_level),
            programming_experience=background.programming_experience,
            robotics_experience=background.robotics_experience,
            math_comfort=background.math_comfort,
            learning_goals=background.learning_goals,
        )

        recommendations = personalization_service.get_content_recommendations(
            background=user_background,
            current_chapter=current_chapter,
        )

        return [
            RecommendationResponse(chapter=r["chapter"], reason=r["reason"])
            for r in recommendations
        ]

    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/preferences/{user_id}", response_model=List[UserPreference])
async def get_user_preferences(user_id: UUID):
    """Get all preferences for a user."""
    uid = str(user_id)
    if uid not in user_preferences:
        return []
    return list(user_preferences[uid].values())


@router.get("/preferences/{user_id}/{chapter_slug}", response_model=UserPreference)
async def get_chapter_preference(user_id: UUID, chapter_slug: str):
    """Get preference for a specific chapter."""
    uid = str(user_id)

    if uid not in user_preferences or chapter_slug not in user_preferences[uid]:
        raise HTTPException(status_code=404, detail="Preference not found")

    return user_preferences[uid][chapter_slug]


@router.put("/preferences/{user_id}/{chapter_slug}", response_model=UserPreference)
async def set_chapter_preference(
    user_id: UUID,
    chapter_slug: str,
    request: SetPreferenceRequest,
):
    """Set or update preference for a chapter."""
    from datetime import datetime

    uid = str(user_id)

    if uid not in user_preferences:
        user_preferences[uid] = {}

    preference = UserPreference(
        id=uuid4(),
        chapter_slug=chapter_slug,
        display_language=request.display_language or "en",
        content_depth=request.content_depth or "default",
        last_visited=datetime.utcnow().isoformat(),
    )

    user_preferences[uid][chapter_slug] = preference
    return preference


@router.delete("/preferences/{user_id}/{chapter_slug}")
async def delete_chapter_preference(user_id: UUID, chapter_slug: str):
    """Delete preference for a chapter."""
    uid = str(user_id)

    if uid in user_preferences and chapter_slug in user_preferences[uid]:
        del user_preferences[uid][chapter_slug]

    return {"status": "deleted", "chapter_slug": chapter_slug}
