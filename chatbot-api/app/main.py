from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.config import settings
from app.routes import chat
from app.services.database_service import init_database
from app.services.qdrant_service import ensure_collection_exists

app = FastAPI(
    title="Physical AI Textbook RAG Chatbot",
    version="1.0.0",
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


@app.on_event("startup")
async def startup_event():
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
