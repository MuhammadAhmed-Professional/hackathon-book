"""
ExplainTerm skill - Reusable agent skill for explaining terms using book context.
"""
import logging
from typing import Dict, Any, Optional
from openai import OpenAI

from src.config import config
from src.services.rag_service import query_qdrant
from src.services.ingestion_service import create_embeddings

logger = logging.getLogger(__name__)

# Initialize OpenAI client
openai_client = OpenAI(api_key=config.OPENAI_API_KEY)


# Persona: Educational Content Explainer
PERSONA = """
You are an educational content explainer specializing in making complex terms and concepts 
accessible to learners. Your goal is to explain terms clearly using context from educational 
materials, with examples and simple language.
"""

# Analytical Questions
ANALYTICAL_QUESTIONS = [
    "What is the term being explained?",
    "How is the term used in the provided context?",
    "What examples or analogies would help understanding?",
    "What related concepts should be mentioned?",
    "What level of detail is appropriate for the audience?"
]

# Decision Principles
DECISION_PRINCIPLES = [
    "Simple Language: Use clear, accessible language avoiding jargon when possible",
    "Book Context: Base explanation on the provided book context, not external knowledge",
    "Examples: Include concrete examples from the context when available",
    "Completeness: Cover the term's meaning, usage, and relevance",
    "Educational: Help learners understand, not just define"
]


def explain_term(term: str, book_context: Optional[str] = None) -> Dict[str, Any]:
    """
    Explain a term using book context and Claude Code Subagents pattern.
    
    Args:
        term: Term to explain
        book_context: Optional context from book (if None, will use RAG to find context)
    
    Returns:
        Dictionary with explanation and metadata
    """
    try:
        # If no context provided, use RAG to find relevant context
        if not book_context:
            # Embed the term
            term_embeddings = create_embeddings([term])
            term_embedding = term_embeddings[0]
            
            # Query Qdrant for relevant chunks
            chunks = query_qdrant(term_embedding, top_k=3)
            
            if chunks:
                # Combine chunks into context
                book_context = "\n\n".join([
                    f"[From {chunk['chapter_id']}]: {chunk['text']}"
                    for chunk in chunks
                ])
            else:
                book_context = "No relevant context found in the book."
        
        # Build prompt using Persona + Questions + Principles
        system_prompt = f"""{PERSONA}

When explaining terms, consider these questions:
{chr(10).join(f"- {q}" for q in ANALYTICAL_QUESTIONS)}

Follow these principles:
{chr(10).join(f"- {p}" for p in DECISION_PRINCIPLES)}"""

        user_prompt = f"""Explain the term "{term}" using the following context from the book.
If the term is not found in the context, clearly state that.

Context from the book:
{book_context}

Term to explain: {term}

Provide a clear explanation:"""

        response = openai_client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.5,
            max_tokens=400
        )
        
        explanation = response.choices[0].message.content
        
        return {
            "term": term,
            "explanation": explanation,
            "context_used": book_context[:200] + "..." if len(book_context) > 200 else book_context,
            "model_used": response.model
        }
    
    except Exception as e:
        logger.error(f"Error in explain_term: {str(e)}")
        raise

