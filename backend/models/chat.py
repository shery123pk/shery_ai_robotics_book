"""
Pydantic models for chatbot endpoints.
Defines request/response schemas for RAG chatbot functionality.
"""

from pydantic import BaseModel, Field
from typing import List, Dict, Any
from datetime import datetime


class Citation(BaseModel):
    """Citation information for chatbot responses."""

    module: str = Field(..., description="Module name (e.g., 'Module 1: ROS 2')")
    chapter: str = Field(..., description="Chapter title")
    section: str | None = Field(None, description="Section title (if available)")


class ChatRequest(BaseModel):
    """Request schema for standard chatbot query."""

    message: str = Field(..., min_length=1, max_length=1000, description="User message/question")
    session_id: str = Field(..., description="Browser session ID (UUID)")
    user_id: str | None = Field(None, description="User ID if logged in (UUID)")


class SelectionRequest(BaseModel):
    """Request schema for text-selection Q&A."""

    selected_text: str = Field(..., min_length=1, max_length=5000, description="Selected text from chapter")
    question: str = Field(..., min_length=1, max_length=1000, description="Question about selected text")
    session_id: str = Field(..., description="Browser session ID (UUID)")
    user_id: str | None = Field(None, description="User ID if logged in (UUID)")


class ChatResponse(BaseModel):
    """Response schema for chatbot queries."""

    response: str = Field(..., description="Chatbot response text")
    citations: List[Citation] = Field(default_factory=list, description="Source citations")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Response timestamp")


class ChatHistoryItem(BaseModel):
    """Single chat message in history."""

    id: str = Field(..., description="Message UUID")
    message: str = Field(..., description="User message")
    response: str = Field(..., description="Chatbot response")
    citations: List[Citation] = Field(default_factory=list, description="Source citations")
    timestamp: datetime = Field(..., description="Message timestamp")


class ChatHistoryResponse(BaseModel):
    """Response schema for chat history endpoint."""

    messages: List[ChatHistoryItem] = Field(default_factory=list, description="List of chat messages")
    total: int = Field(..., description="Total number of messages")
    limit: int = Field(..., description="Pagination limit")
    offset: int = Field(..., description="Pagination offset")
