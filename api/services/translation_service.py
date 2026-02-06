"""Translation service for Urdu translation of chapter content."""

import hashlib
import os
from typing import Optional

from .openai_service import get_openai_service


class TranslationService:
    """Service for translating content to Urdu."""

    # In-memory cache (replace with Redis in production)
    _cache: dict = {}

    def __init__(self):
        self.openai = get_openai_service()

    def _cache_key(self, text: str, target_lang: str) -> str:
        """Generate cache key for translation."""
        content_hash = hashlib.md5(text.encode()).hexdigest()
        return f"{target_lang}:{content_hash}"

    async def translate(
        self,
        text: str,
        target_language: str = "ur",
        preserve_code: bool = True,
        use_cache: bool = True,
    ) -> str:
        """
        Translate text to target language.

        Args:
            text: Text to translate
            target_language: Target language code ('ur' for Urdu)
            preserve_code: Keep code blocks in English
            use_cache: Use cached translation if available

        Returns:
            Translated text
        """
        # Check cache
        if use_cache:
            cache_key = self._cache_key(text, target_language)
            if cache_key in self._cache:
                return self._cache[cache_key]

        # Translate using OpenAI
        translated = await self.openai.translate_text(
            text=text,
            target_language=target_language,
            preserve_code=preserve_code,
        )

        # Cache result
        if use_cache:
            self._cache[cache_key] = translated

        return translated

    async def translate_chapter(
        self,
        content: str,
        target_language: str = "ur",
    ) -> str:
        """
        Translate full chapter content.

        Handles large content by splitting into sections.
        """
        # Split into manageable chunks (by sections)
        sections = content.split("\n## ")

        if len(sections) == 1:
            # Small content, translate directly
            return await self.translate(content, target_language)

        # Translate each section
        translated_sections = []

        # First section (before any ##)
        if sections[0].strip():
            translated = await self.translate(sections[0], target_language)
            translated_sections.append(translated)

        # Remaining sections
        for section in sections[1:]:
            section_content = "## " + section
            translated = await self.translate(section_content, target_language)
            translated_sections.append(translated)

        return "\n".join(translated_sections)

    def clear_cache(self, language: Optional[str] = None) -> int:
        """Clear translation cache."""
        if language:
            keys_to_remove = [k for k in self._cache if k.startswith(f"{language}:")]
            for key in keys_to_remove:
                del self._cache[key]
            return len(keys_to_remove)
        else:
            count = len(self._cache)
            self._cache.clear()
            return count


# Singleton instance
_translation_service: Optional[TranslationService] = None


def get_translation_service() -> TranslationService:
    """Get or create translation service singleton."""
    global _translation_service
    if _translation_service is None:
        _translation_service = TranslationService()
    return _translation_service
