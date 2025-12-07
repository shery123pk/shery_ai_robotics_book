"""
Configuration management for the Physical AI Textbook backend API.
Loads environment variables and provides typed configuration access.
"""

from pydantic_settings import BaseSettings, SettingsConfigDict
from functools import lru_cache


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Application
    app_name: str = "Physical AI Textbook API"
    app_version: str = "1.0.0"
    debug: bool = False

    # OpenAI API
    openai_api_key: str
    openai_model_chat: str = "gpt-4-turbo-preview"
    openai_model_personalize: str = "gpt-3.5-turbo"  # Cheaper for personalization
    openai_model_translate: str = "gpt-3.5-turbo"  # Cheaper for translation
    openai_embedding_model: str = "text-embedding-ada-002"

    # Qdrant Vector Database
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "book_embeddings"

    # Neon Postgres Database
    database_url: str

    # JWT Authentication
    jwt_secret_key: str
    jwt_algorithm: str = "HS256"
    jwt_access_token_expire_minutes: int = 43200  # 30 days

    # Rate Limiting
    rate_limit_requests_per_minute: int = 10

    # CORS
    cors_origins: list[str] = ["*"]  # Configure for production

    # Model configuration
    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
    )


@lru_cache
def get_settings() -> Settings:
    """
    Get cached settings instance.

    Returns:
        Settings: Application settings loaded from environment.
    """
    return Settings()


# Global settings instance
settings = get_settings()
