"""
RAG query endpoint for answering questions with conversation persistence.
"""
from fastapi import APIRouter, HTTPException, Depends, Header
from pydantic import BaseModel
from typing import Optional
import logging

from src.services.rag_service import ask_question
from src.services.db_service import save_conversation, get_user_by_id
from src.models.answer import Answer
from src.models.conversation import ConversationCreate, Citation
from src.services.auth_service import verify_token
from src.services.personalization_service import personalization_service

logger = logging.getLogger(__name__)

router = APIRouter()


class AskRequest(BaseModel):
    """Request model for asking questions."""
    question: str
    session_id: Optional[str] = None


class AskResponse(BaseModel):
    """Response model for ask endpoint."""
    answer_id: str
    answer_text: str
    sources: list
    confidence: float
    conversation_id: Optional[int] = None
    model_used: Optional[str] = None
    error_message: Optional[str] = None


@router.post("/ask", response_model=AskResponse)
async def ask(request: AskRequest, authorization: Optional[str] = Header(None)):
    """
    Answer a question using RAG (Retrieval-Augmented Generation) and persist to Neon Postgres.

    Args:
        request: AskRequest with question text and optional session_id
        authorization: Optional Authorization header with JWT token

    Returns:
        AskResponse with answer, source citations, and conversation_id
    """
    try:
        # Extract user_id from JWT if authenticated
        user_id = None
        user_profile = None
        if authorization and authorization.startswith("Bearer "):
            token = authorization[7:]
            try:
                payload = verify_token(token)
                user_id = payload["user_id"]
                logger.info(f"Authenticated request from user_id={user_id}")

                # Get user profile for personalization
                try:
                    user_data = get_user_by_id(user_id)
                    if user_data:
                        user_profile = {
                            'software_background': user_data.get('software_background', {}),
                            'hardware_background': user_data.get('hardware_background', {})
                        }
                        logger.info(f"User profile loaded for personalization: user_id={user_id}")
                except Exception as profile_error:
                    logger.warning(f"Could not load user profile: {str(profile_error)}")
                    user_profile = None

            except Exception as auth_error:
                logger.warning(f"Invalid token in request: {str(auth_error)}")
                # Continue as anonymous user
                pass

        if not request.question or not request.question.strip():
            raise HTTPException(status_code=400, detail="Question cannot be empty")

        # Get personalization context to enhance RAG query
        personalization_context = ""
        if user_profile:
            personalization_context = personalization_service.get_personalization_context(user_profile)
            logger.info(f"Personalization context: {personalization_context}")

        # Enhance question with personalization context
        enhanced_question = request.question
        if personalization_context:
            enhanced_question = f"{request.question} [Context: {personalization_context}]"

        # Get answer using RAG with personalized context
        answer = ask_question(enhanced_question)

        # Convert SourceCitation objects to Citation objects for database
        citations = []
        for source in answer.sources:
            # Parse chapter_id to extract module and chapter (format: "module1/ros2-architecture")
            chapter_parts = source.chapter_id.split('/')
            module = chapter_parts[0] if len(chapter_parts) > 0 else "unknown"
            chapter = chapter_parts[1] if len(chapter_parts) > 1 else source.chapter_id

            citations.append(Citation(
                module=module,
                chapter=chapter,
                chunk_id=source.chunk_id,
                relevance_score=source.relevance_score
            ))

        # Persist conversation to Neon Postgres
        try:
            conversation = save_conversation(ConversationCreate(
                user_id=user_id,  # Populated from JWT if authenticated
                session_id=request.session_id,
                question=request.question,
                answer=answer.answer_text,
                citations=citations,
                question_type="rag"
            ))
            conversation_id = conversation.id
            logger.info(f"Conversation saved: ID={conversation_id}, session_id={request.session_id}")
        except Exception as db_error:
            logger.error(f"Failed to save conversation to database: {str(db_error)}")
            # Don't fail the request - still return the answer
            conversation_id = None

        # Convert to response format
        return AskResponse(
            answer_id=answer.answer_id,
            answer_text=answer.answer_text,
            sources=[source.to_dict() for source in answer.sources],
            confidence=answer.confidence,
            conversation_id=conversation_id,
            model_used=answer.model_used,
            error_message=answer.error_message
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing question: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error processing your question: {str(e)}"
        )

