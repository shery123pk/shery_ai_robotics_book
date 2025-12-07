"""
Content API endpoints for translation and personalization
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from services.translate_service import translate_to_urdu
from services.personalize_service import personalize_chapter
from utils.rate_limit import is_rate_limited
from utils.logger import get_logger
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


class PersonalizeRequest(BaseModel):
    """Request model for personalization"""
    markdown_content: str
    chapter_id: str
    user_id: Optional[str] = None
    software_experience: str  # "none" | "beginner" | "intermediate" | "advanced"
    hardware_experience: str  # "none" | "hobbyist" | "professional" | "expert"
    session_id: str


class PersonalizeResponse(BaseModel):
    """Response model for personalization"""
    personalized_content: str
    original_length: int
    personalized_length: int
    software_level: str
    hardware_level: str


@router.post("/personalize", response_model=PersonalizeResponse)
async def personalize_content(request: PersonalizeRequest):
    """
    Personalize textbook chapter content based on user's technical background.

    Args:
        request: Personalization request with content and user background

    Returns:
        Personalized content adapted to user's experience level
    """

    # Rate limiting (5 personalizations per minute)
    rate_key = f"personalize:{request.session_id}"
    if is_rate_limited(rate_key, limit=5, window_seconds=60):
        raise HTTPException(
            status_code=429,
            detail="Personalization rate limit exceeded. Please wait before requesting more."
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

        # Validate experience levels
        valid_software = ["none", "beginner", "intermediate", "advanced"]
        valid_hardware = ["none", "hobbyist", "professional", "expert"]

        if request.software_experience not in valid_software:
            raise HTTPException(
                status_code=400,
                detail=f"Invalid software_experience. Must be one of: {valid_software}"
            )

        if request.hardware_experience not in valid_hardware:
            raise HTTPException(
                status_code=400,
                detail=f"Invalid hardware_experience. Must be one of: {valid_hardware}"
            )

        logger.info(
            f"Personalization request: chapter={request.chapter_id}, "
            f"software={request.software_experience}, hardware={request.hardware_experience}"
        )

        # Personalize content
        personalized_text = await personalize_chapter(
            markdown_content=request.markdown_content,
            chapter_id=request.chapter_id,
            user_id=request.user_id,
            software_experience=request.software_experience,
            hardware_experience=request.hardware_experience
        )

        return PersonalizeResponse(
            personalized_content=personalized_text,
            original_length=len(request.markdown_content),
            personalized_length=len(personalized_text),
            software_level=request.software_experience,
            hardware_level=request.hardware_experience
        )

    except Exception as e:
        logger.error(f"Personalization endpoint error: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Personalization failed: {str(e)}"
        )


@router.get("/health")
async def content_health():
    """Health check for content service"""
    return {
        "status": "healthy",
        "services": ["translation", "personalization"]
    }
