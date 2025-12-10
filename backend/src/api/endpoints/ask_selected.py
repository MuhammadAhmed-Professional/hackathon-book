"""
Selected-text question endpoint for answering questions using only provided context,
with conversation persistence to Neon Postgres.
"""
from fastapi import APIRouter, HTTPException, Header
from pydantic import BaseModel
from typing import Optional
import logging
import uuid
from openai import OpenAI

from src.config import config
from src.models.answer import Answer
from src.models.source_citation import SourceCitation
from src.services.db_service import save_conversation
from src.models.conversation import ConversationCreate, Citation
from src.services.auth_service import verify_token

logger = logging.getLogger(__name__)

router = APIRouter()

# Initialize OpenAI client
openai_client = OpenAI(api_key=config.OPENAI_API_KEY)


class AskSelectedRequest(BaseModel):
    """Request model for selected-text questions."""
    question: str
    context: str  # The selected text
    session_id: Optional[str] = None


class AskSelectedResponse(BaseModel):
    """Response model for ask_selected endpoint."""
    answer_id: str
    answer_text: str
    sources: list
    confidence: float
    conversation_id: Optional[int] = None
    model_used: Optional[str] = None
    error_message: Optional[str] = None


def generate_answer_from_context(question: str, context: str) -> dict:
    """
    Generate answer using only the provided context (no RAG query).
    
    Args:
        question: User's question
        context: Selected text context
    
    Returns:
        Dictionary with answer text and model information
    """
    try:
        # Create prompt with only the selected context
        system_prompt = """You are a helpful assistant that answers questions using ONLY the provided context.
Answer questions using ONLY the provided context. If the answer cannot be found in the context, 
clearly state that the information is not available in the provided text. Do not use any external knowledge."""
        
        user_prompt = f"""Context:
{context}

Question: {question}

Please provide an accurate answer based only on the context above. If the answer cannot be found in the context, clearly state that."""
        
        # Call OpenAI API
        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.7,
            max_tokens=500
        )
        
        answer_text = response.choices[0].message.content
        model_used = response.model
        
        logger.info(f"Generated answer from selected text using {model_used}")
        
        return {
            "answer_text": answer_text,
            "model_used": model_used
        }
    
    except Exception as e:
        logger.error(f"Error generating answer from context: {str(e)}")
        raise


@router.post("/ask_selected", response_model=AskSelectedResponse)
async def ask_selected(request: AskSelectedRequest, authorization: Optional[str] = Header(None)):
    """
    Answer a question using only the selected text context (no RAG query).

    Args:
        request: AskSelectedRequest with question and context text
        authorization: Optional Authorization header with JWT token

    Returns:
        AskSelectedResponse with answer
    """
    try:
        # Extract user_id from JWT if authenticated
        user_id = None
        if authorization and authorization.startswith("Bearer "):
            token = authorization[7:]
            try:
                payload = verify_token(token)
                user_id = payload["user_id"]
                logger.info(f"Authenticated selected-text request from user_id={user_id}")
            except Exception as auth_error:
                logger.warning(f"Invalid token in selected-text request: {str(auth_error)}")
                # Continue as anonymous user
                pass
        if not request.question or not request.question.strip():
            raise HTTPException(status_code=400, detail="Question cannot be empty")
        
        if not request.context or not request.context.strip():
            raise HTTPException(status_code=400, detail="Context (selected text) cannot be empty")
        
        # Validate context length (minimum 10 characters)
        if len(request.context.strip()) < 10:
            raise HTTPException(
                status_code=400,
                detail="Selected text is too short. Please select more text to provide context."
            )
        
        # Generate answer using only the selected context
        answer_data = generate_answer_from_context(request.question, request.context)
        
        # Create a simple source citation from the context
        sources = [
            SourceCitation(
                chunk_id="selected_text",
                chapter_id="user_selection",
                relevance_score=1.0,
                snippet=request.context[:200] + "..." if len(request.context) > 200 else request.context
            )
        ]
        
        # Create Answer object
        answer = Answer(
            answer_id=str(uuid.uuid4()),
            answer_text=answer_data["answer_text"],
            sources=sources,
            confidence=1.0,  # High confidence since we're using exact context
            model_used=answer_data["model_used"]
        )

        # Persist conversation to Neon Postgres (T072)
        try:
            citation = Citation(
                module="user_selection",
                chapter="selected_text",
                chunk_id="user_provided",
                relevance_score=1.0
            )

            conversation = save_conversation(ConversationCreate(
                user_id=user_id,  # Populated from JWT if authenticated
                session_id=request.session_id,
                question=request.question,
                answer=answer.answer_text,
                citations=[citation],
                question_type="selected_text"
            ))
            conversation_id = conversation.id
            logger.info(f"Selected-text conversation saved: ID={conversation_id}, session_id={request.session_id}")
        except Exception as db_error:
            logger.error(f"Failed to save selected-text conversation: {str(db_error)}")
            # Don't fail the request - still return the answer
            conversation_id = None

        # Convert to response format
        return AskSelectedResponse(
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
        logger.error(f"Error processing selected-text question: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error processing your question: {str(e)}"
        )

