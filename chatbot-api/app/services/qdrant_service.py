"""Qdrant vector store service."""

import uuid
from typing import Optional

from qdrant_client import QdrantClient
from qdrant_client.http import models

from app.config import settings
from app.services.embedding_service import VECTOR_SIZE, get_embedding

# Initialize Qdrant client lazily
_client = None

COLLECTION_NAME = settings.QDRANT_COLLECTION_NAME


def get_client():
    """Get Qdrant client (lazy initialization)."""
    global _client
    if _client is None:
        if not settings.QDRANT_URL or not settings.QDRANT_API_KEY:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set")
        _client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
        )
    return _client


def ensure_collection_exists():
    """Create collection if it doesn't exist."""
    try:
        client = get_client()
        collections = client.get_collections().collections
        collection_names = [c.name for c in collections]

        if COLLECTION_NAME not in collection_names:
            client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(
                    size=VECTOR_SIZE,
                    distance=models.Distance.COSINE,
                ),
            )
            print(f"Created collection: {COLLECTION_NAME}")
        else:
            print(f"Collection {COLLECTION_NAME} already exists")
    except Exception as e:
        print(f"Qdrant collection check failed: {e}")


def add_document(
    content: str,
    chapter: str,
    section: Optional[str] = None,
    metadata: Optional[dict] = None,
) -> str:
    """Add a document chunk to Qdrant."""
    client = get_client()
    doc_id = str(uuid.uuid4())
    embedding = get_embedding(content)

    payload = {
        "content": content,
        "chapter": chapter,
        "section": section or "",
        **(metadata or {}),
    }

    client.upsert(
        collection_name=COLLECTION_NAME,
        points=[
            models.PointStruct(
                id=doc_id,
                vector=embedding,
                payload=payload,
            )
        ],
    )

    return doc_id


def search_documents(
    query: str, limit: int = 5, chapter_filter: Optional[str] = None
) -> list[dict]:
    """Search for relevant documents."""
    try:
        client = get_client()
        query_embedding = get_embedding(query)

        # Build filter if chapter specified
        search_filter = None
        if chapter_filter:
            search_filter = models.Filter(
                must=[
                    models.FieldCondition(
                        key="chapter",
                        match=models.MatchValue(value=chapter_filter),
                    )
                ]
            )

        results = client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=limit,
            query_filter=search_filter,
        )

        return [
            {
                "id": str(result.id),
                "content": result.payload.get("content", ""),
                "chapter": result.payload.get("chapter", ""),
                "section": result.payload.get("section", ""),
                "score": result.score,
            }
            for result in results
        ]
    except Exception as e:
        print(f"Search failed: {e}")
        return []


def delete_collection():
    """Delete the collection (use with caution)."""
    client = get_client()
    client.delete_collection(collection_name=COLLECTION_NAME)
