"""Database service - optional for serverless deployment."""

import json
from typing import Optional

from app.config import settings

# In-memory storage fallback for serverless
_sessions = {}
_messages = {}
_message_counter = 0

# Try to import psycopg2, but make it optional
try:
    import psycopg2
    from psycopg2.extras import RealDictCursor

    HAS_POSTGRES = bool(settings.DATABASE_URL)
except ImportError:
    HAS_POSTGRES = False


def get_connection():
    """Get database connection."""
    if not HAS_POSTGRES:
        return None
    try:
        return psycopg2.connect(settings.DATABASE_URL)
    except Exception as e:
        print(f"Database connection failed: {e}")
        return None


def init_database():
    """Initialize database tables."""
    if not HAS_POSTGRES:
        print("Running without database (in-memory mode)")
        return

    conn = get_connection()
    if not conn:
        print("Could not connect to database, using in-memory mode")
        return

    try:
        cur = conn.cursor()

        # Create chat_sessions table
        cur.execute("""
            CREATE TABLE IF NOT EXISTS chat_sessions (
                id SERIAL PRIMARY KEY,
                session_id VARCHAR(255) UNIQUE NOT NULL,
                user_id VARCHAR(255),
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
                updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)

        # Create chat_messages table
        cur.execute("""
            CREATE TABLE IF NOT EXISTS chat_messages (
                id SERIAL PRIMARY KEY,
                session_id VARCHAR(255) NOT NULL,
                role VARCHAR(50) NOT NULL,
                content TEXT NOT NULL,
                metadata JSONB,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)

        # Create feedback table
        cur.execute("""
            CREATE TABLE IF NOT EXISTS chat_feedback (
                id SERIAL PRIMARY KEY,
                message_id INTEGER,
                rating INTEGER CHECK (rating >= 1 AND rating <= 5),
                feedback_text TEXT,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)

        conn.commit()
        cur.close()
        conn.close()
        print("Database initialized successfully")
    except Exception as e:
        print(f"Database initialization failed: {e}")


def create_session(session_id: str, user_id: Optional[str] = None) -> dict:
    """Create a new chat session."""
    global _sessions

    conn = get_connection()
    if not conn:
        # In-memory fallback
        _sessions[session_id] = {"session_id": session_id, "user_id": user_id}
        return _sessions[session_id]

    try:
        cur = conn.cursor(cursor_factory=RealDictCursor)
        cur.execute(
            """
            INSERT INTO chat_sessions (session_id, user_id)
            VALUES (%s, %s)
            ON CONFLICT (session_id) DO UPDATE SET updated_at = CURRENT_TIMESTAMP
            RETURNING *
            """,
            (session_id, user_id),
        )
        session = dict(cur.fetchone())
        conn.commit()
        cur.close()
        conn.close()
        return session
    except Exception as e:
        print(f"Create session failed: {e}")
        _sessions[session_id] = {"session_id": session_id, "user_id": user_id}
        return _sessions[session_id]


def save_message(
    session_id: str, role: str, content: str, metadata: Optional[dict] = None
) -> dict:
    """Save a chat message."""
    global _messages, _message_counter

    conn = get_connection()
    if not conn:
        # In-memory fallback
        _message_counter += 1
        msg = {
            "id": _message_counter,
            "session_id": session_id,
            "role": role,
            "content": content,
            "metadata": metadata,
        }
        if session_id not in _messages:
            _messages[session_id] = []
        _messages[session_id].append(msg)
        return msg

    try:
        cur = conn.cursor(cursor_factory=RealDictCursor)
        cur.execute(
            """
            INSERT INTO chat_messages (session_id, role, content, metadata)
            VALUES (%s, %s, %s, %s)
            RETURNING *
            """,
            (session_id, role, content, json.dumps(metadata) if metadata else None),
        )
        message = dict(cur.fetchone())
        conn.commit()
        cur.close()
        conn.close()
        return message
    except Exception as e:
        print(f"Save message failed: {e}")
        _message_counter += 1
        return {
            "id": _message_counter,
            "session_id": session_id,
            "role": role,
            "content": content,
        }


def get_session_messages(session_id: str, limit: int = 20) -> list[dict]:
    """Get messages for a session."""
    conn = get_connection()
    if not conn:
        # In-memory fallback
        return _messages.get(session_id, [])[-limit:]

    try:
        cur = conn.cursor(cursor_factory=RealDictCursor)
        cur.execute(
            """
            SELECT * FROM chat_messages
            WHERE session_id = %s
            ORDER BY created_at DESC
            LIMIT %s
            """,
            (session_id, limit),
        )
        messages = [dict(row) for row in cur.fetchall()]
        cur.close()
        conn.close()
        return list(reversed(messages))
    except Exception as e:
        print(f"Get messages failed: {e}")
        return _messages.get(session_id, [])[-limit:]


def save_feedback(message_id: int, rating: int, feedback_text: Optional[str] = None):
    """Save feedback for a message."""
    conn = get_connection()
    if not conn:
        return  # Skip in memory mode

    try:
        cur = conn.cursor()
        cur.execute(
            """
            INSERT INTO chat_feedback (message_id, rating, feedback_text)
            VALUES (%s, %s, %s)
            """,
            (message_id, rating, feedback_text),
        )
        conn.commit()
        cur.close()
        conn.close()
    except Exception as e:
        print(f"Save feedback failed: {e}")
