"""OpenAI service for embeddings and chat completions."""

import os
from typing import List, Optional

from openai import AsyncOpenAI


class OpenAIService:
    """Service for OpenAI API interactions."""

    def __init__(self):
        self.client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.embedding_model = os.getenv("EMBEDDING_MODEL", "text-embedding-3-small")
        self.chat_model = os.getenv("CHAT_MODEL", "gpt-4o-mini")

    async def create_embedding(self, text: str) -> List[float]:
        """Create embedding for a single text."""
        response = await self.client.embeddings.create(
            model=self.embedding_model,
            input=text,
        )
        return response.data[0].embedding

    async def create_embeddings(self, texts: List[str]) -> List[List[float]]:
        """Create embeddings for multiple texts."""
        response = await self.client.embeddings.create(
            model=self.embedding_model,
            input=texts,
        )
        return [item.embedding for item in response.data]

    async def chat_completion(
        self,
        messages: List[dict],
        temperature: float = 0.7,
        max_tokens: int = 1024,
        system_prompt: Optional[str] = None,
    ) -> tuple[str, int]:
        """
        Generate a chat completion.

        Returns:
            Tuple of (response_content, total_tokens_used)
        """
        if system_prompt:
            messages = [{"role": "system", "content": system_prompt}] + messages

        response = await self.client.chat.completions.create(
            model=self.chat_model,
            messages=messages,
            temperature=temperature,
            max_tokens=max_tokens,
        )

        content = response.choices[0].message.content
        tokens_used = response.usage.total_tokens if response.usage else 0

        return content, tokens_used

    async def translate_text(
        self,
        text: str,
        target_language: str = "ur",
        preserve_code: bool = True,
    ) -> str:
        """Translate text to target language."""
        system_prompt = f"""You are a professional translator. Translate the following text to {target_language}.

Rules:
- Maintain the original meaning and tone
- {"Keep code blocks, technical terms, and variable names in English" if preserve_code else "Translate everything"}
- Preserve markdown formatting
- For Urdu, use proper RTL formatting"""

        messages = [{"role": "user", "content": text}]

        response, _ = await self.chat_completion(
            messages=messages,
            system_prompt=system_prompt,
            temperature=0.3,
            max_tokens=4096,
        )

        return response


# Singleton instance
_openai_service: Optional[OpenAIService] = None


def get_openai_service() -> OpenAIService:
    """Get or create OpenAI service singleton."""
    global _openai_service
    if _openai_service is None:
        _openai_service = OpenAIService()
    return _openai_service
