"""
FastAPI backend for Physical AI & Humanoid Robotics Textbook.
Provides RAG chatbot, authentication, personalization, and translation APIs.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from config import settings

# Initialize FastAPI app
app = FastAPI(
    title=settings.app_name,
    version=settings.app_version,
    description="Backend API for Physical AI & Humanoid Robotics interactive textbook",
    docs_url="/api/docs",
    redoc_url="/api/redoc",
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/")
async def root():
    """Root endpoint - API health check."""
    return {
        "message": "Physical AI & Humanoid Robotics Textbook API",
        "version": settings.app_version,
        "status": "running",
    }


@app.get("/api/health")
async def health_check():
    """Health check endpoint for monitoring."""
    return {
        "status": "healthy",
        "version": settings.app_version,
    }


# Import and include routers
from api import chat

# Register chat router
app.include_router(chat.router, tags=["chat"])

# Additional routers (to be added in later phases)
# from api import auth, content
# app.include_router(auth.router, prefix="/api/auth", tags=["auth"])
# app.include_router(content.router, prefix="/api", tags=["content"])


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=settings.debug,
    )
