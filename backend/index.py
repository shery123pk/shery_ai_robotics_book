"""
Vercel entry point for FastAPI application.
This file exports the FastAPI app for Vercel's Python runtime.
"""

from main import app

# Vercel will use this 'app' variable as the ASGI application
# The variable must be named 'app' for Vercel to recognize it
