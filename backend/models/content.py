"""
Pydantic models for content personalization and translation endpoints.
Defines request/response schemas for chapter adaptation features.
"""

from pydantic import BaseModel, Field
from datetime import datetime


class PersonalizeRequest(BaseModel):
    """Request schema for content personalization."""

    chapter_id: str = Field(..., description="Chapter identifier (e.g., 'module-1/02-nodes-topics')")
    original_markdown: str = Field(..., min_length=1, description="Original chapter markdown content")


class TranslateRequest(BaseModel):
    """Request schema for Urdu translation."""

    chapter_id: str = Field(..., description="Chapter identifier")
    original_markdown: str = Field(..., min_length=1, description="Original chapter markdown content")


class ContentResponse(BaseModel):
    """Response schema for personalization and translation."""

    adapted_markdown: str = Field(..., description="Adapted/translated markdown content")
    cached: bool = Field(default=False, description="Whether content was retrieved from cache")
    generated_at: datetime = Field(default_factory=datetime.utcnow, description="Generation timestamp")
    language: str | None = Field(None, description="Language code (e.g., 'ur' for Urdu)")
