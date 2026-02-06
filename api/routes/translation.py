"""Translation API routes."""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from services.translation_service import get_translation_service

router = APIRouter(prefix="/translate")


class TranslateRequest(BaseModel):
    """Request to translate content."""
    content: str

    content: str
    target_language: str = "ur"
    preserve_code: bool = True
    use_cache: bool = True


class TranslateResponse(BaseModel):
    """Translation response."""

    translation: str
    source_language: str
    target_language: str
    cached: bool = False


class TranslateChapterRequest(BaseModel):
    """Request to translate a full chapter."""

    content: str
    chapter_slug: str
    target_language: str = "ur"


@router.post("/", response_model=TranslateResponse)
async def translate_content(request: TranslateRequest):
    """
    Translate content to target language (Urdu).

    Preserves code blocks in English by default.
    Uses caching to avoid repeated API calls for same content.
    """
    if request.target_language != "ur":
        raise HTTPException(
            status_code=400,
            detail="Only Urdu (ur) translation is currently supported",
        )

    translation_service = get_translation_service()

    try:
        # Check if this will hit cache
        cache_key = translation_service._cache_key(
            request.content, request.target_language
        )
        is_cached = cache_key in translation_service._cache

        translation = await translation_service.translate(
            text=request.content,
            target_language=request.target_language,
            preserve_code=request.preserve_code,
            use_cache=request.use_cache,
        )

        return TranslateResponse(
            translation=translation,
            source_language="en",
            target_language=request.target_language,
            cached=is_cached,
        )

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Translation failed: {str(e)}")


@router.post("/chapter", response_model=TranslateResponse)
async def translate_chapter(request: TranslateChapterRequest):
    """
    Translate a full chapter to target language.

    Handles large content by splitting into sections.
    """
    if request.target_language != "ur":
        raise HTTPException(
            status_code=400,
            detail="Only Urdu (ur) translation is currently supported",
        )

    translation_service = get_translation_service()

    try:
        translation = await translation_service.translate_chapter(
            content=request.content,
            target_language=request.target_language,
        )

        return TranslateResponse(
            translation=translation,
            source_language="en",
            target_language=request.target_language,
        )

    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Chapter translation failed: {str(e)}"


@router.delete("/cache")
async def clear_cache(language: str = None):
    """Clear translation cache."""
    translation_service = get_translation_service()
    count = translation_service.clear_cache(language)

    return {
        "status": "cleared",
        "entries_removed": count,
        "language_filter": language,
    }
