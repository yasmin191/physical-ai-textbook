"""
Physical AI & Humanoid Robotics Textbook - API Server

FastAPI backend for RAG chatbot, authentication, and personalization.
"""

import os
from contextlib import asynccontextmanager
from typing import AsyncGenerator

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import routes
from routes import chat, auth, personalization, translation


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator:
    """Application lifespan handler for startup/shutdown."""
    # Startup
    print("Starting Physical AI Textbook API...")
    yield
    # Shutdown
    print("Shutting down Physical AI Textbook API...")


# Create FastAPI application
app = FastAPI(
    title="Physical AI & Humanoid Robotics Textbook API",
    description="Backend API for the Physical AI textbook with RAG chatbot, authentication, and personalization",
    version="1.0.0",
    lifespan=lifespan,
    docs_url="/docs",
    redoc_url="/redoc",
)

# Configure CORS
allowed_origins = os.getenv("ALLOWED_ORIGINS", "http://localhost:3000").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "PATCH", "OPTIONS"],
    allow_headers=["*"],
)


# Include routers
app.include_router(chat.router, prefix="/api/v1", tags=["Chat"])
app.include_router(auth.router, prefix="/api/v1", tags=["Authentication"])
app.include_router(personalization.router, prefix="/api/v1", tags=["Personalization"])
app.include_router(translation.router, prefix="/api/v1", tags=["Translation"])


@app.get("/")
async def root():
    """Root endpoint - API health check."""
    return {
        "status": "healthy",
        "service": "Physical AI Textbook API",
        "version": "1.0.0",
    }


@app.get("/health")
async def health_check():
    """Health check endpoint for monitoring."""
    return {"status": "ok"}


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=int(os.getenv("PORT", 8000)),
        reload=os.getenv("ENVIRONMENT", "development") == "development",
    )
