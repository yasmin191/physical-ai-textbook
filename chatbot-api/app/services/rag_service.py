from typing import Optional

from openai import OpenAI

from app.config import settings
from app.services.database_service import get_session_messages, save_message
from app.services.qdrant_service import search_documents

client = OpenAI(api_key=settings.OPENAI_API_KEY)

SYSTEM_PROMPT = """You are an expert AI assistant for a Physical AI & Humanoid Robotics textbook.
Your role is to help students understand concepts related to:
- ROS 2 (Robot Operating System)
- Gazebo and Unity simulation
- NVIDIA Isaac platform
- Humanoid robot kinematics and dynamics
- Vision-Language-Action (VLA) models
- Bipedal locomotion and balance control

When answering questions:
1. Use the provided context from the textbook to give accurate answers
2. If the context doesn't contain relevant information, say so clearly
3. Provide code examples when helpful (primarily in Python/ROS 2)
4. Be encouraging and supportive for learners at all levels
5. Reference specific chapters or sections when relevant

Context from the textbook:
{context}

Remember: Only answer based on the provided context. If you don't know something or it's not in the context, be honest about it."""


def build_context(documents: list[dict]) -> str:
    """Build context string from retrieved documents."""
    if not documents:
        return "No relevant content found in the textbook."

    context_parts = []
    for doc in documents:
        chapter = doc.get("chapter", "Unknown")
        section = doc.get("section", "")
        content = doc.get("content", "")

        header = f"[Chapter: {chapter}"
        if section:
            header += f", Section: {section}"
        header += "]"

        context_parts.append(f"{header}\n{content}")

    return "\n\n---\n\n".join(context_parts)


def chat(
    session_id: str,
    user_message: str,
    selected_text: Optional[str] = None,
    chapter_filter: Optional[str] = None,
) -> dict:
    """Process a chat message and return AI response."""

    # If user selected text, use it as additional context
    query = user_message
    if selected_text:
        query = f"Based on this selected text: '{selected_text}'\n\nUser question: {user_message}"

    # Search for relevant documents
    documents = search_documents(query=query, limit=5, chapter_filter=chapter_filter)

    # Build context from retrieved documents
    context = build_context(documents)

    # Get conversation history
    history = get_session_messages(session_id, limit=10)

    # Build messages for OpenAI
    messages = [{"role": "system", "content": SYSTEM_PROMPT.format(context=context)}]

    # Add conversation history
    for msg in history:
        messages.append({"role": msg["role"], "content": msg["content"]})

    # Add current user message
    messages.append({"role": "user", "content": user_message})

    # Save user message to database
    save_message(
        session_id,
        "user",
        user_message,
        {"selected_text": selected_text, "chapter_filter": chapter_filter},
    )

    # Call OpenAI
    response = client.chat.completions.create(
        model=settings.CHAT_MODEL, messages=messages, temperature=0.7, max_tokens=1000
    )

    assistant_message = response.choices[0].message.content

    # Save assistant message to database
    saved_msg = save_message(
        session_id,
        "assistant",
        assistant_message,
        {
            "sources": [
                {"chapter": d["chapter"], "score": d["score"]} for d in documents
            ]
        },
    )

    return {
        "message": assistant_message,
        "message_id": saved_msg["id"],
        "sources": [
            {
                "chapter": doc["chapter"],
                "section": doc["section"],
                "relevance": round(doc["score"] * 100, 1),
            }
            for doc in documents
        ],
    }


def answer_selected_text(selected_text: str, question: Optional[str] = None) -> dict:
    """Answer a question about selected text without session context."""

    # Search for related content to provide additional context
    search_query = selected_text[:500]  # Limit search query length
    documents = search_documents(query=search_query, limit=3)

    context = build_context(documents)

    # Build the prompt
    if question:
        user_content = f'The user selected this text:\n\n"{selected_text}"\n\nTheir question is: {question}'
    else:
        user_content = f'The user selected this text and wants you to explain it:\n\n"{selected_text}"\n\nPlease explain this concept clearly.'

    messages = [
        {"role": "system", "content": SYSTEM_PROMPT.format(context=context)},
        {"role": "user", "content": user_content},
    ]

    response = client.chat.completions.create(
        model=settings.CHAT_MODEL, messages=messages, temperature=0.7, max_tokens=800
    )

    return {
        "message": response.choices[0].message.content,
        "sources": [
            {
                "chapter": doc["chapter"],
                "section": doc["section"],
                "relevance": round(doc["score"] * 100, 1),
            }
            for doc in documents
        ],
    }
