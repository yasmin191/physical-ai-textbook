# Minimal imports to avoid startup crashes
import os

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

# Get frontend URLs from environment
frontend_urls = os.getenv(
    "FRONTEND_URLS",
    "http://localhost:3000,https://yasmin191.github.io,https://physical-ai-textbook.vercel.app",
).split(",")

app = FastAPI(
    title="Physical AI Textbook RAG Chatbot",
    version="1.0.0",
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all for debugging
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


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


@app.get("/api/v1/test")
async def test():
    return {"message": "API is working"}


# Lazy load the chat routes to avoid import errors at startup
@app.post("/api/v1/chat/")
async def chat_endpoint(request: dict):
    """Chat endpoint with lazy loading."""
    try:
        from app.services.rag_service import chat

        message = request.get("message", "")
        session_id = request.get("session_id", "default")
        selected_text = request.get("selected_text")

        result = chat(session_id, message, selected_text)
        return result
    except Exception as e:
        return {
            "message": f"I apologize, but I encountered an error: {str(e)}. The RAG service may still be initializing.",
            "sources": [],
            "error": str(e),
        }


@app.post("/api/v1/chat/selected-text")
async def selected_text_endpoint(request: dict):
    """Answer questions about selected text."""
    try:
        from app.services.rag_service import answer_selected_text

        selected_text = request.get("selected_text", "")
        question = request.get("question")

        result = answer_selected_text(selected_text, question)
        return result
    except Exception as e:
        return {
            "message": f"I apologize, but I encountered an error: {str(e)}",
            "sources": [],
            "error": str(e),
        }


# Handler for Vercel serverless
try:
    from mangum import Mangum

    handler = Mangum(app)
except ImportError:
    handler = None
