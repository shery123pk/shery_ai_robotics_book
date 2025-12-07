"""
Neon Serverless Postgres database connection using asyncpg.
Provides async connection pool for user data, chat history, and personalized content.
"""

import asyncpg
from typing import Optional
from config import settings


# Global connection pool
_pool: Optional[asyncpg.Pool] = None


async def get_db_pool() -> asyncpg.Pool:
    """
    Get or create the database connection pool.

    Returns:
        asyncpg.Pool: Database connection pool
    """
    global _pool

    if _pool is None:
        _pool = await asyncpg.create_pool(
            dsn=settings.database_url,
            min_size=1,
            max_size=10,
            command_timeout=60,
        )

    return _pool


async def close_db_pool():
    """Close the database connection pool."""
    global _pool

    if _pool is not None:
        await _pool.close()
        _pool = None


async def execute_query(query: str, *args):
    """
    Execute a query and return results.

    Args:
        query: SQL query string
        *args: Query parameters

    Returns:
        Query results
    """
    pool = await get_db_pool()
    async with pool.acquire() as connection:
        return await connection.fetch(query, *args)


async def execute_one(query: str, *args):
    """
    Execute a query and return a single row.

    Args:
        query: SQL query string
        *args: Query parameters

    Returns:
        Single row result or None
    """
    pool = await get_db_pool()
    async with pool.acquire() as connection:
        return await connection.fetchrow(query, *args)


async def execute_write(query: str, *args):
    """
    Execute a write query (INSERT, UPDATE, DELETE).

    Args:
        query: SQL query string
        *args: Query parameters

    Returns:
        Number of affected rows
    """
    pool = await get_db_pool()
    async with pool.acquire() as connection:
        return await connection.execute(query, *args)
