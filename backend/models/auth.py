"""
Pydantic models for authentication endpoints.
Defines request/response schemas for signup and signin.
"""

from pydantic import BaseModel, EmailStr, Field
from typing import Optional
from datetime import datetime


class UserCreate(BaseModel):
    """Request schema for user signup."""

    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., min_length=8, description="User password (min 8 characters)")
    full_name: str = Field(..., min_length=2, max_length=100, description="User's full name")
    role: Optional[str] = Field(default="student", description="User role (student, instructor, researcher)")
    background: Optional[str] = Field(default=None, description="Educational/professional background")


class UserLogin(BaseModel):
    """Request schema for user login."""

    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., description="User password")


class UserResponse(BaseModel):
    """Response schema for user information."""

    id: str = Field(..., description="User UUID")
    email: str = Field(..., description="User email address")
    full_name: str = Field(..., description="User's full name")
    role: str = Field(..., description="User role")
    background: Optional[str] = Field(None, description="User background")
    created_at: datetime = Field(..., description="Account creation timestamp")


class TokenResponse(BaseModel):
    """Response schema for authentication (signup/login)."""

    access_token: str = Field(..., description="JWT access token")
    token_type: str = Field(default="bearer", description="Token type")
    user: UserResponse = Field(..., description="User information")


# Legacy aliases for backward compatibility
SignUpRequest = UserCreate
SignInRequest = UserLogin
UserProfile = UserResponse
