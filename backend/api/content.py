"""
Content API endpoints for translation and personalization
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from backend.services.translate_service import translate_to_urdu
from backend.utils.rate_limit import is_rate_limited
from backend.utils.logger import get_logger
from typing import Optional

router = APIRouter(prefix="/api/content", tags=["content"])
logger = get_logger(__name__)


class TranslateRequest(BaseModel):
    """Request model for translation"""
    markdown_content: str
    chapter_id: Optional[str] = None
    session_id: str


class TranslateResponse(BaseModel):
    """Response model for translation"""
    translated_content: str
    original_length: int
    translated_length: int


@router.post("/translate", response_model=TranslateResponse)
async def translate_content(request: TranslateRequest):
    """
    Translate textbook chapter content to Urdu.

    Args:
        request: Translation request with markdown content

    Returns:
        Translated Urdu content with RTL formatting
    """

    # Rate limiting (3 translations per minute)
    rate_key = f"translate:{request.session_id}"
    if is_rate_limited(rate_key, limit=3, window_seconds=60):
        raise HTTPException(
            status_code=429,
            detail="Translation rate limit exceeded. Please wait before translating again."
        )

    try:
        # Validate content length
        if len(request.markdown_content) > 50000:
            raise HTTPException(
                status_code=400,
                detail="Content too long. Maximum 50,000 characters."
            )

        if len(request.markdown_content) < 10:
            raise HTTPException(
                status_code=400,
                detail="Content too short. Minimum 10 characters."
            )

        logger.info(f"Translation request for {len(request.markdown_content)} chars")

        # Translate to Urdu
        translated_text = await translate_to_urdu(request.markdown_content)

        return TranslateResponse(
            translated_content=translated_text,
            original_length=len(request.markdown_content),
            translated_length=len(translated_text)
        )

    except Exception as e:
        logger.error(f"Translation endpoint error: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Translation failed: {str(e)}"
        )


@router.get("/health")
async def content_health():
    """Health check for content service"""
    return {
        "status": "healthy",
        "services": ["translation", "personalization"]
    }
