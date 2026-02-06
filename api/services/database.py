"""Database service for Neon Postgres connections."""

import os
from contextlib import asynccontextmanager
from typing import Optional

import asyncpg


class DatabaseService:
    """Service for PostgreSQL database operations."""

    def __init__(self):
        self.database_url = os.getenv("DATABASE_URL")
        self.pool: Optional[asyncpg.Pool] = None

    async def connect(self) -> None:
        """Create connection pool."""
        if self.database_url:
            self.pool = await asyncpg.create_pool(
                self.database_url,
                min_size=2,
                max_size=10,
            )

    async def disconnect(self) -> None:
        """Close connection pool."""
        if self.pool:
            await self.pool.close()

    @asynccontextmanager
    async def get_connection(self):
        """Get a connection from the pool."""
        if not self.pool:
            await self.connect()

        async with self.pool.acquire() as connection:
            yield connection

    async def execute(self, query: str, *args) -> str:
        """Execute a query and return status."""
        async with self.get_connection() as conn:
            return await conn.execute(query, *args)

    async def fetch(self, query: str, *args) -> list:
        """Fetch multiple rows."""
        async with self.get_connection() as conn:
            return await conn.fetch(query, *args)

    async def fetchrow(self, query: str, *args) -> Optional[dict]:
        """Fetch a single row."""
        async with self.get_connection() as conn:
            row = await conn.fetchrow(query, *args)
            return dict(row) if row else None

    async def fetchval(self, query: str, *args):
        """Fetch a single value."""
        async with self.get_connection() as conn:
            return await conn.fetchval(query, *args)

    # User operations
    async def create_user(
        self,
        email: str,
        password_hash: str,
        name: str,
        background: dict,
    ) -> dict:
        """Create a new user."""
        query = """
            INSERT INTO users (email, password_hash, name, background)
            VALUES ($1, $2, $3, $4)
            RETURNING id, email, name, background, created_at
        """
        return await self.fetchrow(query, email, password_hash, name, background)

    async def get_user_by_email(self, email: str) -> Optional[dict]:
        """Get user by email."""
        query = "SELECT * FROM users WHERE email = $1"
        return await self.fetchrow(query, email)

    async def get_user_by_id(self, user_id: str) -> Optional[dict]:
        """Get user by ID."""
        query = "SELECT * FROM users WHERE id = $1"
        return await self.fetchrow(query, user_id)

    # Session operations
    async def create_session(self, user_id: Optional[str] = None) -> dict:
        """Create a new chat session."""
        query = """
            INSERT INTO chat_sessions (user_id)
            VALUES ($1)
            RETURNING id, user_id, created_at
        """
        return await self.fetchrow(query, user_id)

    async def get_session(self, session_id: str) -> Optional[dict]:
        """Get session by ID."""
        query = "SELECT * FROM chat_sessions WHERE id = $1"
        return await self.fetchrow(query, session_id)

    async def get_session_messages(self, session_id: str) -> list:
        """Get all messages in a session."""
        query = """
            SELECT * FROM chat_messages
            WHERE session_id = $1
            ORDER BY created_at ASC
        """
        return await self.fetch(query, session_id)

    async def add_message(
        self,
        session_id: str,
        role: str,
        content: str,
        sources: Optional[list] = None,
    ) -> dict:
        """Add a message to a session."""
        query = """
            INSERT INTO chat_messages (session_id, role, content, sources)
            VALUES ($1, $2, $3, $4)
            RETURNING id, session_id, role, content, sources, created_at
        """
        import json

        sources_json = json.dumps(sources) if sources else None
        return await self.fetchrow(query, session_id, role, content, sources_json)

    # Preference operations
    async def get_user_preferences(self, user_id: str) -> Optional[dict]:
        """Get user preferences."""
        query = "SELECT * FROM user_preferences WHERE user_id = $1"
        return await self.fetchrow(query, user_id)

    async def update_user_preferences(
        self,
        user_id: str,
        preferences: dict,
    ) -> dict:
        """Update user preferences."""
        import json

        query = """
            INSERT INTO user_preferences (user_id, preferences)
            VALUES ($1, $2)
            ON CONFLICT (user_id)
            DO UPDATE SET preferences = $2, updated_at = NOW()
            RETURNING *
        """
        return await self.fetchrow(query, user_id, json.dumps(preferences))


# Singleton instance
_database_service: Optional[DatabaseService] = None


def get_database_service() -> DatabaseService:
    """Get or create database service singleton."""
    global _database_service
    if _database_service is None:
        _database_service = DatabaseService()
    return _database_service
