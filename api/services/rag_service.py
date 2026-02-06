"""RAG (Retrieval Augmented Generation) service for the chatbot."""

from dataclasses import dataclass
from typing import List, Optional

from .embedding_service import get_embedding_service
from .openai_service import get_openai_service
from .qdrant_service import get_qdrant_service


@dataclass
class Source:
    """A source reference from the textbook."""

    chapter_slug: str
    section_title: str
    chapter_title: str
    relevance_score: float
    text_snippet: str


@dataclass
class RAGResponse:
    """Response from RAG service."""

    content: str
    sources: List[Source]
    tokens_used: int
    language: str


class RAGService:
    """Service for Retrieval Augmented Generation."""

    SYSTEM_PROMPT_EN = """You are an expert tutor for the Physical AI & Humanoid Robotics course.
Your role is to help students understand concepts from the textbook.

Guidelines:
- Provide accurate, educational answers based on the provided context
- Cite specific chapters when referencing content
- If the context doesn't contain relevant information, say so honestly
- Use clear, beginner-friendly explanations when appropriate
- Include code examples when helpful (use proper formatting)
- Encourage further exploration of related topics

Context from the textbook:
{context}

Remember: Only answer based on the provided context. If unsure, ask for clarification."""

    SYSTEM_PROMPT_UR = """آپ فزیکل اے آئی اور ہیومنائیڈ روبوٹکس کورس کے ماہر استاد ہیں۔
آپ کا کردار طلباء کو نصابی کتاب کے تصورات سمجھنے میں مدد کرنا ہے۔

ہدایات:
- فراہم کردہ سیاق و سباق کی بنیاد پر درست، تعلیمی جوابات دیں
- مواد کا حوالہ دیتے وقت مخصوص ابواب کا حوالہ دیں
- اگر سیاق و سباق میں متعلقہ معلومات نہیں ہیں تو ایمانداری سے کہیں
- جب مناسب ہو تو واضح، ابتدائی سطح کی وضاحتیں استعمال کریں
- جب مددگار ہو تو کوڈ کی مثالیں شامل کریں

نصابی کتاب سے سیاق و سباق:
{context}

یاد رکھیں: صرف فراہم کردہ سیاق و سباق کی بنیاد پر جواب دیں۔"""

    def __init__(self):
        self.openai = get_openai_service()
        self.qdrant = get_qdrant_service()
        self.embedding = get_embedding_service()

    async def answer_question(
        self,
        question: str,
        language: str = "en",
        selected_text: Optional[str] = None,
        chapter_context: Optional[str] = None,
        top_k: int = 5,
    ) -> RAGResponse:
        """
        Answer a question using RAG.

        Args:
            question: User's question
            language: Response language ('en' or 'ur')
            selected_text: Optional highlighted text for context
            chapter_context: Optional chapter slug to focus search
            top_k: Number of chunks to retrieve

        Returns:
            RAGResponse with answer, sources, and token usage
        """
        # Build search query
        search_query = question
        if selected_text:
            search_query = f"{question}\n\nContext: {selected_text}"

        # Get query embedding
        query_embedding = await self.embedding.embed_query(search_query)

        # Search for relevant chunks
        search_results = await self.qdrant.search(
            query_embedding=query_embedding,
            limit=top_k,
            score_threshold=0.4,
            chapter_filter=chapter_context,
        )

        # Build context from search results
        context_parts = []
        sources = []
        seen_chapters = set()

        for result in search_results:
            context_parts.append(
                f"[{result['chapter_title']} - {result['section_title']}]\n{result['text']}"
            )

            # Track unique sources
            source_key = f"{result['chapter_slug']}:{result['section_title']}"
            if source_key not in seen_chapters:
                sources.append(
                    Source(
                        chapter_slug=result["chapter_slug"],
                        section_title=result["section_title"],
                        chapter_title=result["chapter_title"],
                        relevance_score=result["score"],
                        text_snippet=result["text"][:200] + "..."
                        if len(result["text"]) > 200
                        else result["text"],
                    )
                )
                seen_chapters.add(source_key)

        context = (
            "\n\n---\n\n".join(context_parts)
            if context_parts
            else "No relevant content found."
        )

        # Select system prompt based on language
        system_prompt = (
            self.SYSTEM_PROMPT_UR if language == "ur" else self.SYSTEM_PROMPT_EN
        )
        system_prompt = system_prompt.format(context=context)

        # Build messages
        messages = [{"role": "user", "content": question}]

        if selected_text:
            messages[0]["content"] = (
                f'Regarding this text: "{selected_text}"\n\nQuestion: {question}'
            )

        # Generate response
        response_content, tokens_used = await self.openai.chat_completion(
            messages=messages,
            system_prompt=system_prompt,
            temperature=0.7,
            max_tokens=1024,
        )

        return RAGResponse(
            content=response_content,
            sources=sources,
            tokens_used=tokens_used,
            language=language,
        )

    async def get_chapter_summary(
        self,
        chapter_slug: str,
        language: str = "en",
    ) -> RAGResponse:
        """Generate a summary for a specific chapter."""
        # Search for all chunks in the chapter
        # Use a generic query to get representative chunks
        query_embedding = await self.embedding.embed_query(
            f"main concepts and key topics"
        )

        search_results = await self.qdrant.search(
            query_embedding=query_embedding,
            limit=10,
            score_threshold=0.3,
            chapter_filter=chapter_slug,
        )

        if not search_results:
            return RAGResponse(
                content="No content found for this chapter.",
                sources=[],
                tokens_used=0,
                language=language,
            )

        # Build context
        context = "\n\n".join([r["text"] for r in search_results])

        prompt = "Please provide a concise summary of this chapter's main topics and key learning points."
        if language == "ur":
            prompt = "براہ کرم اس باب کے اہم موضوعات اور کلیدی سیکھنے کے نکات کا مختصر خلاصہ فراہم کریں۔"

        system_prompt = f"""Summarize the following chapter content in a clear, educational manner.
Focus on the main concepts, key takeaways, and practical applications.

Chapter content:
{context}"""

        response_content, tokens_used = await self.openai.chat_completion(
            messages=[{"role": "user", "content": prompt}],
            system_prompt=system_prompt,
            temperature=0.5,
            max_tokens=512,
        )

        return RAGResponse(
            content=response_content,
            sources=[
                Source(
                    chapter_slug=chapter_slug,
                    section_title="Summary",
                    chapter_title=search_results[0]["chapter_title"]
                    if search_results
                    else chapter_slug,
                    relevance_score=1.0,
                    text_snippet="Chapter summary",
                )
            ],
            tokens_used=tokens_used,
            language=language,
        )

    def detect_language(self, text: str) -> str:
        """Simple language detection (Urdu vs English)."""
        # Check for Urdu characters (Arabic script range)
        urdu_chars = sum(1 for c in text if "\u0600" <= c <= "\u06ff")
        total_alpha = sum(1 for c in text if c.isalpha())

        if total_alpha == 0:
            return "en"

        urdu_ratio = urdu_chars / total_alpha
        return "ur" if urdu_ratio > 0.3 else "en"


# Singleton instance
_rag_service: Optional[RAGService] = None


def get_rag_service() -> RAGService:
    """Get or create RAG service singleton."""
    global _rag_service
    if _rag_service is None:
        _rag_service = RAGService()
    return _rag_service
