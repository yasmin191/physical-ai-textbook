"""Qdrant vector database service for semantic search."""

import os
from typing import List, Optional
from uuid import uuid4

from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, PointStruct, VectorParams


class QdrantService:
    """Service for Qdrant vector database operations."""

    COLLECTION_NAME = "textbook_chapters"
    VECTOR_SIZE = 1536  # text-embedding-3-small dimension

    def __init__(self):
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        if qdrant_url and qdrant_api_key:
            self.client = QdrantClient(
                url=qdrant_url,
                api_key=qdrant_api_key,
            )
        else:
            # Use local in-memory storage for development
            self.client = QdrantClient(":memory:")

    async def ensure_collection(self) -> None:
        """Create collection if it doesn't exist."""
        collections = self.client.get_collections().collections
        collection_names = [c.name for c in collections]

        if self.COLLECTION_NAME not in collection_names:
            self.client.create_collection(
                collection_name=self.COLLECTION_NAME,
                vectors_config=VectorParams(
                    size=self.VECTOR_SIZE,
                    distance=Distance.COSINE,
                ),
            )

    async def upsert_chunks(
        self,
        chunks: List[dict],
        embeddings: List[List[float]],
    ) -> int:
        """
        Upsert text chunks with embeddings.

        Args:
            chunks: List of dicts with keys: id, text, chapter_slug, section_title, metadata
            embeddings: Corresponding embeddings for each chunk

        Returns:
            Number of points upserted
        """
        await self.ensure_collection()

        points = []
        for chunk, embedding in zip(chunks, embeddings):
            point = PointStruct(
                id=chunk.get("id", str(uuid4())),
                vector=embedding,
                payload={
                    "text": chunk["text"],
                    "chapter_slug": chunk["chapter_slug"],
                    "section_title": chunk.get("section_title", ""),
                    "chapter_title": chunk.get("chapter_title", ""),
                    "module": chunk.get("module", ""),
                    **chunk.get("metadata", {}),
                },
            )
            points.append(point)

        self.client.upsert(
            collection_name=self.COLLECTION_NAME,
            points=points,
        )

        return len(points)

    async def search(
        self,
        query_embedding: List[float],
        limit: int = 5,
        score_threshold: float = 0.5,
        chapter_filter: Optional[str] = None,
    ) -> List[dict]:
        """
        Search for similar chunks.

        Args:
            query_embedding: Query vector
            limit: Maximum number of results
            score_threshold: Minimum similarity score
            chapter_filter: Optional filter by chapter slug

        Returns:
            List of matching chunks with scores
        """
        filter_conditions = None
        if chapter_filter:
            filter_conditions = models.Filter(
                must=[
                    models.FieldCondition(
                        key="chapter_slug",
                        match=models.MatchValue(value=chapter_filter),
                    )
                ]
            )

        results = self.client.search(
            collection_name=self.COLLECTION_NAME,
            query_vector=query_embedding,
            limit=limit,
            score_threshold=score_threshold,
            query_filter=filter_conditions,
        )

        return [
            {
                "id": str(hit.id),
                "score": hit.score,
                "text": hit.payload.get("text", ""),
                "chapter_slug": hit.payload.get("chapter_slug", ""),
                "section_title": hit.payload.get("section_title", ""),
                "chapter_title": hit.payload.get("chapter_title", ""),
                "module": hit.payload.get("module", ""),
            }
            for hit in results
        ]

    async def delete_by_chapter(self, chapter_slug: str) -> int:
        """Delete all chunks for a specific chapter."""
        result = self.client.delete(
            collection_name=self.COLLECTION_NAME,
            points_selector=models.FilterSelector(
                filter=models.Filter(
                    must=[
                        models.FieldCondition(
                            key="chapter_slug",
                            match=models.MatchValue(value=chapter_slug),
                        )
                    ]
                )
            ),
        )
        return result.status

    async def get_collection_info(self) -> dict:
        """Get collection statistics."""
        try:
            info = self.client.get_collection(self.COLLECTION_NAME)
            return {
                "name": self.COLLECTION_NAME,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "status": info.status,
            }
        except Exception as e:
            return {"error": str(e)}


# Singleton instance
_qdrant_service: Optional[QdrantService] = None


def get_qdrant_service() -> QdrantService:
    """Get or create Qdrant service singleton."""
    global _qdrant_service
    if _qdrant_service is None:
        _qdrant_service = QdrantService()
    return _qdrant_service
