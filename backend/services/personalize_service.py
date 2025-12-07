"""
Content personalization service for adapting chapters to user background
"""

from openai import OpenAI
from config import settings
from utils.logger import get_logger
from database.postgres import get_db_pool
from typing import Optional, Dict
import hashlib
from datetime import datetime, timedelta

logger = get_logger(__name__)
openai_client = OpenAI(api_key=settings.openai_api_key)

# In-memory cache for personalized content (7-day TTL)
_personalization_cache: Dict[str, tuple[str, datetime]] = {}
CACHE_TTL_DAYS = 7


async def personalize_chapter(
    markdown_content: str,
    chapter_id: str,
    user_id: Optional[str],
    software_experience: str,
    hardware_experience: str
) -> str:
    """
    Personalize chapter content based on user's technical background.

    Args:
        markdown_content: Original chapter markdown
        chapter_id: Unique chapter identifier
        user_id: User ID (optional, for caching)
        software_experience: "none" | "beginner" | "intermediate" | "advanced"
        hardware_experience: "none" | "hobbyist" | "professional" | "expert"

    Returns:
        Personalized markdown content
    """

    try:
        # Generate cache key
        cache_key = _generate_cache_key(chapter_id, software_experience, hardware_experience)

        # Check cache
        cached_content = _get_from_cache(cache_key)
        if cached_content:
            logger.info(f"Returning cached personalized content for {chapter_id}")
            return cached_content

        logger.info(
            f"Personalizing chapter {chapter_id} for "
            f"software:{software_experience}, hardware:{hardware_experience}"
        )

        # Build personalization prompt based on experience levels
        personalization_instructions = _build_personalization_prompt(
            software_experience, hardware_experience
        )

        # System prompt
        system_prompt = f"""You are an expert educational content adapter for robotics and AI courses.
        Adapt the following textbook chapter to match the student's technical background.

        CRITICAL RULES:
        1. Keep ALL code blocks unchanged (between ``` markers)
        2. Keep YAML frontmatter unchanged (between --- markers)
        3. Keep markdown structure (headings, lists, links) unchanged
        4. Keep technical terms and product names unchanged (ROS 2, Isaac, etc.)
        5. ONLY modify explanations and descriptions

        {personalization_instructions}

        Preserve the exact same structure and code examples.
        Output ONLY the adapted markdown, nothing else."""

        # User prompt
        user_prompt = f"""Adapt this chapter for the student's background:

{markdown_content}

Remember: Keep code blocks and structure unchanged, only adapt explanations."""

        # Call GPT-3.5-turbo (cheaper than GPT-4, sufficient for this task)
        completion = openai_client.chat.completions.create(
            model="gpt-3.5-turbo-16k",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.5,  # Moderate creativity
            max_tokens=8000
        )

        personalized_text = completion.choices[0].message.content

        # Store in cache
        _store_in_cache(cache_key, personalized_text)

        # Store in database if user is authenticated
        if user_id:
            try:
                await _store_in_database(
                    user_id, chapter_id, personalized_text,
                    software_experience, hardware_experience
                )
            except Exception as e:
                logger.error(f"Failed to store personalized content in DB: {e}")
                # Don't fail the request if DB storage fails

        logger.info(f"Personalization complete: {len(personalized_text)} characters")

        return personalized_text

    except Exception as e:
        logger.error(f"Personalization failed: {e}")
        # Return original content on error
        logger.warning("Returning original content due to personalization error")
        return markdown_content


def _build_personalization_prompt(software_exp: str, hardware_exp: str) -> str:
    """Build personalization instructions based on experience levels."""

    prompts = {
        "software": {
            "none": "Add inline definitions for programming terms. Explain basic concepts like variables, functions, classes.",
            "beginner": "Briefly explain programming concepts. Add comments to code explaining what each part does.",
            "intermediate": "Assume familiarity with Python/C++. Skip basic programming explanations. Add advanced tips.",
            "advanced": "Assume expert-level programming. Skip all basic explanations. Include performance optimization tips and best practices.",
        },
        "hardware": {
            "none": "Explain hardware components in detail. Define terms like 'sensor', 'actuator', 'GPIO'. Include diagrams.",
            "hobbyist": "Assume basic hardware knowledge (Arduino, Raspberry Pi). Explain robotics-specific components.",
            "professional": "Assume professional hardware experience. Skip basic electronics. Focus on robotics-specific details.",
            "expert": "Assume expert hardware knowledge. Skip explanations of standard components. Include advanced integration details.",
        }
    }

    software_prompt = prompts["software"].get(software_exp, prompts["software"]["beginner"])
    hardware_prompt = prompts["hardware"].get(hardware_exp, prompts["hardware"]["hobbyist"])

    return f"""
PERSONALIZATION RULES:
- Software Level ({software_exp}): {software_prompt}
- Hardware Level ({hardware_exp}): {hardware_prompt}

Adjust explanations to match these levels while keeping code and structure unchanged.
"""


def _generate_cache_key(chapter_id: str, software_exp: str, hardware_exp: str) -> str:
    """Generate deterministic cache key for content."""
    key_string = f"{chapter_id}:{software_exp}:{hardware_exp}"
    return hashlib.md5(key_string.encode()).hexdigest()


def _get_from_cache(cache_key: str) -> Optional[str]:
    """Retrieve personalized content from cache if not expired."""
    if cache_key in _personalization_cache:
        content, timestamp = _personalization_cache[cache_key]
        if datetime.now() - timestamp < timedelta(days=CACHE_TTL_DAYS):
            return content
        else:
            # Expired, remove from cache
            del _personalization_cache[cache_key]
    return None


def _store_in_cache(cache_key: str, content: str):
    """Store personalized content in cache with timestamp."""
    _personalization_cache[cache_key] = (content, datetime.now())


async def _store_in_database(
    user_id: str,
    chapter_id: str,
    personalized_content: str,
    software_exp: str,
    hardware_exp: str
):
    """Store personalized content in database for future reference."""
    pool = await get_db_pool()
    async with pool.acquire() as conn:
        await conn.execute(
            """
            INSERT INTO personalized_content (user_id, chapter_id, content, software_experience, hardware_experience, created_at)
            VALUES ($1, $2, $3, $4, $5, $6)
            ON CONFLICT (user_id, chapter_id)
            DO UPDATE SET
                content = EXCLUDED.content,
                software_experience = EXCLUDED.software_experience,
                hardware_experience = EXCLUDED.hardware_experience,
                created_at = EXCLUDED.created_at
            """,
            user_id,
            chapter_id,
            personalized_content,
            software_exp,
            hardware_exp,
            datetime.now()
        )
