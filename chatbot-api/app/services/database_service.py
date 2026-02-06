import json
from datetime import datetime
from typing import Optional

import psycopg2
from psycopg2.extras import RealDictCursor


def get_connection():
    """Get database connection."""
    return psycopg2.connect(settings.DATABASE_URL)


def init_database():
    """Initialize database tables."""
    conn = get_connection()
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
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
            FOREIGN KEY (session_id) REFERENCES chat_sessions(session_id)
        )
    """)

    # Create feedback table
    cur.execute("""
        CREATE TABLE IF NOT EXISTS chat_feedback (
            id SERIAL PRIMARY KEY,
            message_id INTEGER REFERENCES chat_messages(id),
            rating INTEGER CHECK (rating >= 1 AND rating <= 5),
            feedback_text TEXT,
            created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
        )
    """)

    conn.commit()
    cur.close()
    conn.close()
    print("Database initialized successfully")


def create_session(session_id: str, user_id: Optional[str] = None) -> dict:
    """Create a new chat session."""
    conn = get_connection()
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


def save_message(
    session_id: str, role: str, content: str, metadata: Optional[dict] = None
) -> dict:
    """Save a chat message."""
    conn = get_connection()
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


def get_session_messages(session_id: str, limit: int = 20) -> list[dict]:
    """Get messages for a session."""
    conn = get_connection()
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


def save_feedback(message_id: int, rating: int, feedback_text: Optional[str] = None):
    """Save feedback for a message."""
    conn = get_connection()
    cur = conn.cursor()

    cur.execute(
        """
        INSERT INTO chat_feedback (message_id, rating, feedback_text)
        VALUES (%s, %s, %s)
        """,
        (message_id, rating, feedback_text),
        VALUES (%s, %s, %s)
        """,
        (message_id, rating, feedback_text)
    )

    conn.commit()
    cur.close()
    conn.close()
