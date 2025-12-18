"""
Vercel serverless function handler for FastAPI app.
This file exposes the FastAPI app as a Vercel serverless function using Mangum adapter.
"""

import sys
from pathlib import Path

# Add parent directory to path so we can import main
backend_dir = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(backend_dir))

from main import app
from mangum import Mangum

# Mangum adapter converts ASGI app (FastAPI) to work with AWS Lambda-style serverless platforms
# Vercel's Python runtime uses a similar interface
handler = Mangum(app, lifespan="off")
