"""Chat API routes for RAG chatbot."""

from datetime import datetime
from typing import List, Optional
from uuid import UUID, uuid4

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from services.rag_service import get_rag_service

router = APIRouter(prefix="/chat")


class ChatRequest(BaseModel):
    """Request model for chat messages."""
    message: str

    message: str
    session_id: Optional[UUID] = None
    language: str = "en"
    selected_text: Optional[str] = None
    chapter_context: Optional[str] = None


class Source(BaseModel):
    """Source reference from textbook."""

    chapter_slug: str
    section_title: str
    chapter_title: str
    relevance_score: float
    text_snippet: Optional[str] = None


class ChatResponse(BaseModel):
    """Response model for chat messages."""

    message_id: UUID
    content: str
    language: str
    sources: List[Source]
    tokens_used: int


class SessionCreate(BaseModel):
    """Request to create a new session."""

    user_id: Optional[UUID] = None


class Session(BaseModel):
    """Chat session model."""

    id: UUID
    session_token: str
    created_at: str


class Message(BaseModel):
    """Chat message model."""

    id: UUID
    role: str
    content: str
    sources: Optional[List[Source]] = None
    created_at: str


# In-memory session storage (replace with database in production)
sessions: dict = {}
session_messages: dict = {}


@router.post("/", response_model=ChatResponse)
async def send_message(request: ChatRequest):
    """
    Send a message to the RAG chatbot.

    The chatbot will search the textbook content and generate
    a contextual response with source citations.
    """
    rag_service = get_rag_service()

    try:
        # Detect language if auto
        language = request.language
        if language == "auto":
            language = rag_service.detect_language(request.message)

        # Get RAG response
        response = await rag_service.answer_question(
            question=request.message,
            language=language,
            selected_text=request.selected_text,
            chapter_context=request.chapter_context,
        )

        # Convert sources to response model
        sources = [
            Source(
                chapter_slug=s.chapter_slug,
                section_title=s.section_title,
                chapter_title=s.chapter_title,
                relevance_score=s.relevance_score,
                text_snippet=s.text_snippet,
            )
            for s in response.sources
        ]

        # Store in session if provided
        message_id = uuid4()
        if request.session_id:
            session_id = str(request.session_id)
            if session_id not in session_messages:
                session_messages[session_id] = []

            # Add user message
            session_messages[session_id].append(
                {
                    "id": str(uuid4()),
                    "role": "user",
                    "content": request.message,
                    "created_at": datetime.utcnow().isoformat(),
                }
            )

            # Add assistant response
            session_messages[session_id].append(
                {
                    "id": str(message_id),
                    "role": "assistant",
                    "content": response.content,
                    "sources": [s.dict() for s in sources],
                    "created_at": datetime.utcnow().isoformat(),
                }
            )

        return ChatResponse(
            message_id=message_id,
            content=response.content,
            language=response.language,
            sources=sources,
            tokens_used=response.tokens_used,
        )

    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error processing chat request: {str(e)}"
        )


@router.post("/sessions", response_model=Session)
async def create_session(request: SessionCreate = None):
    """Create a new chat session."""
    session_id = uuid4()
    session_token = str(uuid4())
    created_at = datetime.utcnow().isoformat()

    sessions[str(session_id)] = {
        "id": str(session_id),
        "token": session_token,
        "user_id": str(request.user_id) if request and request.user_id else None,
        "created_at": created_at,
    }
    session_messages[str(session_id)] = []

    return Session(
        id=session_id,
        session_token=session_token,
        created_at=created_at,
    )


@router.get("/sessions/{session_id}")
async def get_session(session_id: UUID):
    """Get chat session with message history."""
    sid = str(session_id)

    if sid not in sessions:
        raise HTTPException(status_code=404, detail="Session not found")

    return {
        "session": sessions[sid],
        "messages": session_messages.get(sid, []),
    }


@router.delete("/sessions/{session_id}")
async def delete_session(session_id: UUID):
    """End and delete a chat session."""
    sid = str(session_id)

    if sid in sessions:
        del sessions[sid]
    if sid in session_messages:
        del session_messages[sid]

    return {"status": "deleted", "session_id": str(session_id)}


@router.get("/summary/{chapter_slug}")
async def get_chapter_summary(chapter_slug: str, language: str = "en"):
    """Get AI-generated summary for a chapter."""
    rag_service = get_rag_service()

    try:
        response = await rag_service.get_chapter_summary(
            chapter_slug=chapter_slug,
            language=language,
        )

        return {
            "chapter_slug": chapter_slug,
            "summary": response.content,
            "language": response.language,
            "tokens_used": response.tokens_used,
        }

    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Error generating summary: {str(e)}"
