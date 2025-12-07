"""
Rate limiting utilities for API endpoints.
Implements simple in-memory rate limiting based on client IP or user ID.
"""

from fastapi import Request, HTTPException
from collections import defaultdict
from datetime import datetime, timedelta
from typing import Callable
from config import settings


# In-memory storage for rate limiting
# Format: {key: [(timestamp1, timestamp2, ...)]}
_rate_limit_store = defaultdict(list)


def get_rate_limit_key(request: Request, user_id: str | None = None) -> str:
    """
    Generate a rate limit key based on user ID or IP address.

    Args:
        request: FastAPI request object
        user_id: Optional user ID (if authenticated)

    Returns:
        Rate limit key string
    """
    if user_id:
        return f"user:{user_id}"
    else:
        client_ip = request.client.host if request.client else "unknown"
        return f"ip:{client_ip}"


def is_rate_limited(key: str, limit: int, window_seconds: int = 60) -> bool:
    """
    Check if a key has exceeded the rate limit.

    Args:
        key: Rate limit key
        limit: Maximum number of requests allowed
        window_seconds: Time window in seconds

    Returns:
        True if rate limited, False otherwise
    """
    now = datetime.utcnow()
    window_start = now - timedelta(seconds=window_seconds)

    # Clean old timestamps
    _rate_limit_store[key] = [
        ts for ts in _rate_limit_store[key] if ts > window_start
    ]

    # Check if limit exceeded
    return len(_rate_limit_store[key]) >= limit


def record_request(key: str):
    """
    Record a request for rate limiting.

    Args:
        key: Rate limit key
    """
    _rate_limit_store[key].append(datetime.utcnow())


async def rate_limit_dependency(request: Request, user_id: str | None = None):
    """
    FastAPI dependency for rate limiting.

    Args:
        request: FastAPI request object
        user_id: Optional user ID (from auth dependency)

    Raises:
        HTTPException: 429 if rate limit exceeded
    """
    key = get_rate_limit_key(request, user_id)
    limit = settings.rate_limit_requests_per_minute

    if is_rate_limited(key, limit):
        raise HTTPException(
            status_code=429,
            detail={
                "error": "Rate limit exceeded. Please try again in 60 seconds.",
                "retry_after": 60,
            },
        )

    record_request(key)


def rate_limit(limit: int = None):
    """
    Decorator for rate limiting endpoints.

    Args:
        limit: Custom rate limit (defaults to config value)

    Returns:
        Decorator function
    """

    def decorator(func: Callable):
        async def wrapper(*args, **kwargs):
            # Extract request from kwargs
            request = kwargs.get("request")
            user_id = kwargs.get("user_id")

            if request:
                key = get_rate_limit_key(request, user_id)
                req_limit = limit or settings.rate_limit_requests_per_minute

                if is_rate_limited(key, req_limit):
                    raise HTTPException(
                        status_code=429,
                        detail="Rate limit exceeded. Please try again later.",
                    )

                record_request(key)

            return await func(*args, **kwargs)

        return wrapper

    return decorator
