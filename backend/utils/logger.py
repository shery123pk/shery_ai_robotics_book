"""
Structured logging setup for the FastAPI backend.
Provides consistent logging format across all modules.
"""

import logging
import sys
from datetime import datetime


# Configure logging format
LOG_FORMAT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
DATE_FORMAT = "%Y-%m-%d %H:%M:%S"


def setup_logger(name: str, level: int = logging.INFO) -> logging.Logger:
    """
    Set up a logger with consistent formatting.

    Args:
        name: Logger name (typically __name__)
        level: Logging level (default: INFO)

    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name)
    logger.setLevel(level)

    # Avoid duplicate handlers
    if not logger.handlers:
        # Console handler
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(level)

        # Formatter
        formatter = logging.Formatter(LOG_FORMAT, DATE_FORMAT)
        console_handler.setFormatter(formatter)

        logger.addHandler(console_handler)

    return logger


# Default logger for the application
app_logger = setup_logger("physical_ai_backend")

# Alias for compatibility
get_logger = setup_logger


def log_api_request(endpoint: str, method: str, user_id: str | None = None):
    """
    Log an API request.

    Args:
        endpoint: API endpoint path
        method: HTTP method
        user_id: Optional user ID
    """
    user_info = f"user_id={user_id}" if user_id else "anonymous"
    app_logger.info(f"{method} {endpoint} - {user_info}")


def log_api_error(endpoint: str, error: Exception, user_id: str | None = None):
    """
    Log an API error.

    Args:
        endpoint: API endpoint path
        error: Exception that occurred
        user_id: Optional user ID
    """
    user_info = f"user_id={user_id}" if user_id else "anonymous"
    app_logger.error(f"Error in {endpoint} - {user_info}: {str(error)}")
