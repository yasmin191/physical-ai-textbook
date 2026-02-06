from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from mangum import Mangum

from app.config import settings
from app.routes import chat
from app.services.database_service import init_database
from app.services.qdrant_service import ensure_collection_exists


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Initialize services on startup."""
    try:
        init_database()
        print("Database initialized")
    except Exception as e:
        print(f"Database initialization failed: {e}")

    try:
        ensure_collection_exists()
        print("Qdrant collection ready")
    except Exception as e:
        print(f"Qdrant initialization failed: {e}")

    yield


app = FastAPI(
    title="Physical AI Textbook RAG Chatbot",
    version="1.0.0",
    lifespan=lifespan,
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.FRONTEND_URLS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routes
app.include_router(chat.router, prefix="/api/v1")


@app.get("/")
async def root():
    return {
        "name": "Physical AI Textbook RAG Chatbot",
        "version": "1.0.0",
        "status": "running",
    }


@app.get("/health")
async def health():
    return {"status": "ok"}


# Handler for Vercel serverless
handler = Mangum(app)
