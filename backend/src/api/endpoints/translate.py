"""
Translation API endpoints for multilingual content support.
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Optional
from src.services.translation_service import translation_service

router = APIRouter(prefix="/translate", tags=["translation"])


class TranslateRequest(BaseModel):
    """Request model for translation endpoint."""
    content: str = Field(..., min_length=1, description="Content to translate")
    target_language: str = Field(default="ur", description="Target language code (ur, es, fr, etc.)")
    preserve_code: bool = Field(default=True, description="Preserve code blocks during translation")
    use_cache: bool = Field(default=True, description="Use translation cache if available")


class TranslateResponse(BaseModel):
    """Response model for translation endpoint."""
    translated: str
    source_language: str
    target_language: str
    language_name: str
    cached: bool
    tokens_used: Optional[int] = None
    glossary: list


class LanguageInfo(BaseModel):
    """Language information model."""
    code: str
    name: str
    native: str
    rtl: bool


@router.post("", response_model=TranslateResponse)
async def translate_content(request: TranslateRequest):
    """
    Translate content to target language while preserving technical terms and code.

    This endpoint uses OpenAI GPT-4o-mini to provide high-quality translations
    of technical robotics content. It preserves:
    - Code blocks (Python, C++, JavaScript, etc.)
    - Technical terminology (ROS 2, DDS, quaternion, etc.)
    - Markdown formatting
    - URLs and file paths

    **Supported Languages**:
    - ur: Urdu (اردو)
    - es: Spanish (Español)
    - fr: French (Français)
    - de: German (Deutsch)
    - zh: Chinese (中文)
    - ar: Arabic (العربية)
    - ja: Japanese (日本語)
    - ko: Korean (한국어)
    - pt: Portuguese (Português)
    - ru: Russian (Русский)

    **Example Request**:
    ```json
    {
        "content": "# ROS 2 Nodes\\n\\nA node is an independent process...",
        "target_language": "ur",
        "preserve_code": true,
        "use_cache": true
    }
    ```

    **Example Response**:
    ```json
    {
        "translated": "# ROS 2 نوڈز\\n\\nایک نوڈ ایک آزاد process ہے...",
        "source_language": "en",
        "target_language": "ur",
        "language_name": "Urdu",
        "cached": false,
        "tokens_used": 450,
        "glossary": [
            {"english": "node", "translated": "نوڈ", "preserve": false}
        ]
    }
    ```
    """
    try:
        result = translation_service.translate(
            content=request.content,
            target_language=request.target_language,
            preserve_code=request.preserve_code,
            use_cache=request.use_cache
        )

        return TranslateResponse(**result)

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Translation failed: {str(e)}"
        )


@router.get("/languages", response_model=list[LanguageInfo])
async def get_supported_languages():
    """
    Get list of supported languages for translation.

    Returns information about each supported language including:
    - Language code (ISO 639-1)
    - English name
    - Native name
    - Whether language is RTL (right-to-left)

    **Example Response**:
    ```json
    [
        {
            "code": "ur",
            "name": "Urdu",
            "native": "اردو",
            "rtl": true
        },
        {
            "code": "es",
            "name": "Spanish",
            "native": "Español",
            "rtl": false
        }
    ]
    ```
    """
    languages = translation_service.get_supported_languages()
    return [LanguageInfo(**lang) for lang in languages]


@router.post("/batch", response_model=list[TranslateResponse])
async def translate_batch(
    requests: list[TranslateRequest]
):
    """
    Translate multiple content items in a single request.

    Useful for translating entire chapters or multiple sections at once.
    Processes translations sequentially to avoid rate limits.

    **Note**: For large batches (>10 items), consider using pagination
    or implementing a queue-based system.

    **Example Request**:
    ```json
    [
        {
            "content": "## Section 1\\nContent...",
            "target_language": "ur"
        },
        {
            "content": "## Section 2\\nMore content...",
            "target_language": "ur"
        }
    ]
    ```
    """
    if len(requests) > 20:
        raise HTTPException(
            status_code=400,
            detail="Maximum 20 items per batch request. Please split into smaller batches."
        )

    results = []
    for request in requests:
        try:
            result = translation_service.translate(
                content=request.content,
                target_language=request.target_language,
                preserve_code=request.preserve_code,
                use_cache=request.use_cache
            )
            results.append(TranslateResponse(**result))
        except Exception as e:
            # Continue with other translations even if one fails
            results.append(TranslateResponse(
                translated=f"[Translation failed: {str(e)}]",
                source_language="en",
                target_language=request.target_language,
                language_name="Unknown",
                cached=False,
                glossary=[]
            ))

    return results


@router.delete("/cache")
async def clear_translation_cache():
    """
    Clear the translation cache.

    Use this endpoint to force fresh translations or free up memory.
    Requires no authentication (for hackathon demo).

    **Note**: In production, this should require admin authentication.
    """
    translation_service.translation_cache.clear()
    return {"message": "Translation cache cleared successfully"}


@router.get("/stats")
async def get_translation_stats():
    """
    Get translation service statistics.

    Returns information about cache size, supported languages, etc.
    """
    return {
        "cache_size": len(translation_service.translation_cache),
        "supported_languages": len(translation_service.get_supported_languages()),
        "default_language": "ur",
        "preserve_terms_count": len(translation_service.translation_cache)
    }
