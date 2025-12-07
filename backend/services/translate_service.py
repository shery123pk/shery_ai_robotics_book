"""
Translation service for converting textbook content to Urdu
"""

from openai import OpenAI
from backend.config import settings
from backend.utils.logger import get_logger
from typing import Optional

logger = get_logger(__name__)
openai_client = OpenAI(api_key=settings.openai_api_key)


async def translate_to_urdu(markdown_content: str, preserve_code: bool = True) -> str:
    """
    Translate textbook chapter to Urdu (Pakistan) using GPT-4.

    Args:
        markdown_content: Original English markdown text
        preserve_code: If True, keep code blocks in English

    Returns:
        Translated Urdu markdown text
    """

    try:
        logger.info(f"Translating {len(markdown_content)} characters to Urdu")

        # System prompt for translation
        system_prompt = """You are an expert translator specializing in technical educational content.
        Translate the following technical textbook content from English to Urdu (Pakistan).

        CRITICAL RULES:
        1. Keep ALL code blocks in English (between ``` markers)
        2. Keep technical terms in English: ROS 2, NVIDIA Isaac, Gazebo, URDF, rclpy, Docker, Python, etc.
        3. Keep command-line commands in English
        4. Translate explanations, descriptions, and learning content to formal academic Urdu
        5. Preserve ALL markdown formatting (headings #, lists -, bold **, links, etc.)
        6. Keep YAML frontmatter unchanged (between --- markers)
        7. Keep URLs and file paths in English
        8. Use formal/academic Urdu style suitable for university textbooks
        9. Maintain the same structure and hierarchy
        10. Keep mathematical notation and symbols unchanged

        Output ONLY the translated markdown, nothing else."""

        # User prompt with content
        user_prompt = f"""Translate this markdown content to Urdu:

{markdown_content}

Remember: Keep code blocks, technical terms, and markdown syntax in English."""

        # Call GPT-4 for translation (using GPT-3.5-turbo for cost optimization)
        completion = openai_client.chat.completions.create(
            model="gpt-3.5-turbo-16k",  # Larger context for full chapters
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.3,  # Lower temperature for accurate translation
            max_tokens=8000   # Allow for longer translations
        )

        translated_text = completion.choices[0].message.content

        logger.info(f"Translation complete: {len(translated_text)} characters")

        return translated_text

    except Exception as e:
        logger.error(f"Translation failed: {e}")
        raise Exception(f"Translation error: {str(e)}")


async def detect_language(text: str) -> str:
    """
    Detect if text is in English or Urdu.

    Args:
        text: Text to analyze

    Returns:
        Language code: 'en' or 'ur'
    """
    # Simple heuristic: check for Urdu Unicode range
    urdu_chars = sum(1 for char in text if '\u0600' <= char <= '\u06FF')
    total_chars = len(text)

    if urdu_chars > total_chars * 0.3:
        return 'ur'
    else:
        return 'en'
