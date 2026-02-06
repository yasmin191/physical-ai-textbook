import uuid
from typing import Optional

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from app.services.database_service import (
    create_session,
    get_session_messages,
    save_feedback,
)

router = APIRouter(prefix="/chat", tags=["chat"])


class ChatRequest(BaseModel):
    session_id: Optional[str] = None
    message: str
    selected_text: Optional[str] = None
    chapter_filter: Optional[str] = None
    user_id: Optional[str] = None


class SelectedTextRequest(BaseModel):
    selected_text: str
    question: Optional[str] = None


class FeedbackRequest(BaseModel):
    message_id: int
    rating: int
    feedback_text: Optional[str] = None


class ChatResponse(BaseModel):
    session_id: str
    message: str
    message_id: int
    sources: list[dict]


class SelectedTextResponse(BaseModel):
    message: str
    sources: list[dict]


@router.post("/", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    """Send a message to the chatbot."""
    try:
        # Create or use existing session
        session_id = request.session_id or str(uuid.uuid4())
        create_session(session_id, request.user_id)

        # Get response from RAG service
        response = chat(
            session_id=session_id,
            user_message=request.message,
            selected_text=request.selected_text,
            chapter_filter=request.chapter_filter,
        )

        return ChatResponse(
            session_id=session_id,
            message=response["message"],
            message_id=response["message_id"],
            sources=response["sources"],
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/selected-text", response_model=SelectedTextResponse)
async def selected_text_endpoint(request: SelectedTextRequest):
    """Answer a question about selected text."""
    try:
        response = answer_selected_text(
            selected_text=request.selected_text, question=request.question
        )

        return SelectedTextResponse(
            message=response["message"], sources=response["sources"]
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/history/{session_id}")
async def get_history(session_id: str, limit: int = 20):
    """Get chat history for a session."""
    try:
        messages = get_session_messages(session_id, limit)
        return {"session_id": session_id, "messages": messages}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

    try:
        messages = get_session_messages(session_id, limit)
        return {"session_id": session_id, "messages": messages}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/feedback")
async def submit_feedback(request: FeedbackRequest):
    """Submit feedback for a message."""
    try:
        save_feedback(request.message_id, request.rating, request.feedback_text)
        return {"success": True}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
