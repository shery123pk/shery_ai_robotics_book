"""
Pydantic models for authentication endpoints.
Defines request/response schemas for signup and signin.
"""

from pydantic import BaseModel, EmailStr, Field
from typing import Literal
from datetime import datetime


class SignUpRequest(BaseModel):
    """Request schema for user signup with background questionnaire."""

    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., min_length=8, description="User password (min 8 characters)")
    software_experience: Literal["beginner", "intermediate", "advanced"] = Field(
        ..., description="Software development experience level"
    )
    hardware_experience: Literal["none", "hobbyist", "professional"] = Field(
        ..., description="Hardware/robotics experience level"
    )


class SignInRequest(BaseModel):
    """Request schema for user signin."""

    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., description="User password")


class TokenResponse(BaseModel):
    """Response schema for authentication (signup/signin)."""

    user_id: str = Field(..., description="User UUID")
    email: str = Field(..., description="User email address")
    access_token: str = Field(..., description="JWT access token")
    token_type: str = Field(default="bearer", description="Token type")
    expires_in: int = Field(..., description="Token expiration time in seconds")


class UserProfile(BaseModel):
    """User profile information."""

    user_id: str = Field(..., description="User UUID")
    email: str = Field(..., description="User email address")
    software_experience: str = Field(..., description="Software experience level")
    hardware_experience: str = Field(..., description="Hardware experience level")
    created_at: datetime = Field(..., description="Account creation timestamp")
    last_login: datetime | None = Field(None, description="Last login timestamp")
