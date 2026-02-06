"""Embedding service for text chunking and vectorization."""

import re
from dataclasses import dataclass
from typing import List, Optional

from .openai_service import get_openai_service


@dataclass
class TextChunk:
    """A chunk of text with metadata."""

    id: str
    text: str
    chapter_slug: str
    section_title: str
    chapter_title: str
    module: str
    start_char: int
    end_char: int


class EmbeddingService:
    """Service for text chunking and embedding generation."""

    DEFAULT_CHUNK_SIZE = 1000
    DEFAULT_CHUNK_OVERLAP = 200

    def __init__(
        self,
        chunk_size: int = DEFAULT_CHUNK_SIZE,
        chunk_overlap: int = DEFAULT_CHUNK_OVERLAP,
    ):
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        self.openai = get_openai_service()

    def chunk_markdown(
        self,
        content: str,
        chapter_slug: str,
        chapter_title: str,
        module: str,
    ) -> List[TextChunk]:
        """
        Split markdown content into semantic chunks.

        Respects section boundaries when possible.
        """
        chunks = []

        # Split by headers first
        sections = self._split_by_headers(content)

        chunk_id = 0
        for section_title, section_content in sections:
            # Clean content
            clean_content = self._clean_markdown(section_content)

            if not clean_content.strip():
                continue

            # Split section into smaller chunks if needed
            section_chunks = self._split_text(
                clean_content,
                self.chunk_size,
                self.chunk_overlap,
            )

            for i, (text, start, end) in enumerate(section_chunks):
                chunk = TextChunk(
                    id=f"{chapter_slug}_{chunk_id}",
                    text=text,
                    chapter_slug=chapter_slug,
                    section_title=section_title,
                    chapter_title=chapter_title,
                    module=module,
                    start_char=start,
                    end_char=end,
                )
                chunks.append(chunk)
                chunk_id += 1

        return chunks

    def _split_by_headers(self, content: str) -> List[tuple[str, str]]:
        """Split content by markdown headers."""
        # Match headers (## or ###)
        header_pattern = r"^(#{2,3})\s+(.+)$"

        sections = []
        current_section = "Introduction"
        current_content = []

        for line in content.split("\n"):
            match = re.match(header_pattern, line)
            if match:
                # Save previous section
                if current_content:
                    sections.append((current_section, "\n".join(current_content)))

                current_section = match.group(2).strip()
                current_content = []
            else:
                current_content.append(line)

        # Don't forget the last section
        if current_content:
            sections.append((current_section, "\n".join(current_content)))

        return sections

    def _clean_markdown(self, text: str) -> str:
        """Clean markdown formatting for embedding."""
        # Remove code blocks but keep inline code
        text = re.sub(r"```[\s\S]*?```", "[code block]", text)

        # Remove images
        text = re.sub(r"!\[.*?\]\(.*?\)", "", text)

        # Simplify links to just text
        text = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", text)

        # Remove HTML tags
        text = re.sub(r"<[^>]+>", "", text)

        # Normalize whitespace
        text = re.sub(r"\n{3,}", "\n\n", text)
        text = re.sub(r" {2,}", " ", text)

        return text.strip()

    def _split_text(
        self,
        text: str,
        chunk_size: int,
        overlap: int,
    ) -> List[tuple[str, int, int]]:
        """
        Split text into overlapping chunks.

        Returns list of (text, start_char, end_char) tuples.
        """
        if len(text) <= chunk_size:
            return [(text, 0, len(text))]

        chunks = []
        start = 0

        while start < len(text):
            end = start + chunk_size

            # Try to break at sentence boundary
            if end < len(text):
                # Look for sentence end in last 20% of chunk
                search_start = end - int(chunk_size * 0.2)
                search_text = text[search_start:end]

                # Find last sentence boundary
                for sep in [". ", ".\n", "! ", "? ", "\n\n"]:
                    last_sep = search_text.rfind(sep)
                    if last_sep != -1:
                        end = search_start + last_sep + len(sep)
                        break

            chunk_text = text[start:end].strip()
            if chunk_text:
                chunks.append((chunk_text, start, end))

            start = end - overlap
            if start >= len(text) - overlap:
                break

        return chunks

    async def embed_chunks(
        self,
        chunks: List[TextChunk],
        batch_size: int = 100,
    ) -> List[List[float]]:
        """Generate embeddings for chunks in batches."""
        all_embeddings = []

        for i in range(0, len(chunks), batch_size):
            batch = chunks[i : i + batch_size]
            texts = [f"{c.chapter_title} - {c.section_title}: {c.text}" for c in batch]

            embeddings = await self.openai.create_embeddings(texts)
            all_embeddings.extend(embeddings)

        return all_embeddings

    async def embed_query(self, query: str) -> List[float]:
        """Generate embedding for a search query."""
        return await self.openai.create_embedding(query)


# Singleton instance
_embedding_service: Optional[EmbeddingService] = None


def get_embedding_service() -> EmbeddingService:
    """Get or create embedding service singleton."""
    global _embedding_service
    if _embedding_service is None:
        _embedding_service = EmbeddingService()
    return _embedding_service
