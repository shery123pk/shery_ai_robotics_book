"""
Vercel serverless function handler for FastAPI app.
This file exposes the FastAPI app as a Vercel serverless function using Mangum adapter.
"""

import sys
import os
from pathlib import Path

# Add parent directory to path so we can import main
backend_dir = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(backend_dir))

# Import app with error handling
try:
    from main import app
    from mangum import Mangum

    # Mangum adapter converts ASGI app (FastAPI) to work with serverless platforms
    handler = Mangum(app, lifespan="off")

except Exception as e:
    # Fallback error handler if initialization fails
    print(f"ERROR initializing app: {e}", file=sys.stderr)
    import traceback
    traceback.print_exc()

    def handler(event, context):
        return {
            "statusCode": 500,
            "body": f"Application initialization error: {str(e)}"
        }
