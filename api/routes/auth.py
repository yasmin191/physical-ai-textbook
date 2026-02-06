"""Authentication API routes."""

from typing import Optional
from uuid import UUID, uuid4

from fastapi import APIRouter, HTTPException, status
from pydantic import BaseModel, EmailStr

router = APIRouter(prefix="/auth")


class SignupRequest(BaseModel):

    email: EmailStr
    password: str
    python_level: str  # none, beginner, intermediate, advanced
    robotics_experience: str  # none, hobbyist, academic, professional
    math_level: str  # basic, calculus, linear_algebra, advanced


class SigninRequest(BaseModel):
    """Request model for user signin."""

    email: EmailStr
    password: str


class User(BaseModel):
    """User model."""

    id: UUID
    email: str
    email_verified: bool
    python_level: str
    robotics_experience: str
    math_level: str


class AuthResponse(BaseModel):
    """Authentication response with tokens."""

    user: User
    access_token: str
    refresh_token: str
    expires_in: int


class TokenRefreshRequest(BaseModel):
    """Request to refresh access token."""

    refresh_token: str


class PasswordResetRequest(BaseModel):
    """Request for password reset."""

    email: EmailStr


class PasswordResetConfirm(BaseModel):
    """Confirm password reset with token."""

    token: str
    new_password: str


@router.post(
    "/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED
)
@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def signup(request: SignupRequest):
    """Create a new user account."""
    # TODO: Implement user creation with better-auth
    user = User(
        id=uuid4(),
        email=request.email,
        email_verified=False,
        python_level=request.python_level,
        robotics_experience=request.robotics_experience,
        math_level=request.math_level,
    )

    return AuthResponse(
        user=user,
        access_token="placeholder_access_token",
        refresh_token="placeholder_refresh_token",
        expires_in=3600,
    )


@router.post("/signin", response_model=AuthResponse)
async def signin(request: SigninRequest):
    """Sign in to existing account."""
    # TODO: Implement authentication
    raise HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail="Authentication not yet implemented",
    )


@router.post("/signout")
async def signout():
    """Sign out and end session."""
    return {"success": True}


@router.post("/refresh")
async def refresh_token(request: TokenRefreshRequest):
    """Refresh access token."""
    # TODO: Implement token refresh
    return {
        "access_token": "new_placeholder_token",
        "expires_in": 3600,
    }


@router.post("/password/forgot")
async def forgot_password(request: PasswordResetRequest):
    """Request password reset email."""
    # TODO: Implement password reset email
    return {"message": "If an account exists, a reset email has been sent"}


@router.post("/password/reset")
async def reset_password(request: PasswordResetConfirm):
    """Reset password with token."""
    # TODO: Implement password reset
    return {"success": True}
