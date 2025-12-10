"""
RAG service for querying Qdrant and generating answers using OpenAI.
"""
import logging
import uuid
from typing import List, Dict, Any
from openai import OpenAI
from qdrant_client import QdrantClient

from src.config import config
from src.models.answer import Answer
from src.models.source_citation import SourceCitation

logger = logging.getLogger(__name__)

# Initialize OpenAI client
openai_client = OpenAI(api_key=config.OPENAI_API_KEY)


def query_qdrant(query_embedding: List[float], top_k: int = 5, collection_name: str = "book_chunks") -> List[Dict[str, Any]]:
    """
    Query Qdrant for similar chunks using vector similarity search.
    
    Args:
        query_embedding: Embedding vector of the query
        top_k: Number of top results to return (default: 5)
        collection_name: Name of the Qdrant collection
    
    Returns:
        List of similar chunks with their metadata and scores
    """
    try:
        # Initialize Qdrant client
        qdrant_client = QdrantClient(
            url=config.QDRANT_URL,
            api_key=config.QDRANT_API_KEY
        )
        
        # Search for similar vectors
        # For qdrant-client 1.7+, try different API methods
        query_response = None
        
        # Approach 1: Try query method (if available)
        try:
            if hasattr(qdrant_client, 'query'):
                query_response = qdrant_client.query(
                    collection_name=collection_name,
                    query_vector=query_embedding,
                    limit=top_k
                )
                logger.info("Query succeeded with query() method")
        except (AttributeError, TypeError) as e1:
            logger.debug(f"query() method not available: {str(e1)}")
        
        # Approach 2: Try query_points with vector as list
        if query_response is None:
            try:
                query_response = qdrant_client.query_points(
                    collection_name=collection_name,
                    query=query_embedding,
                    limit=top_k
                )
                logger.info("Query succeeded with query_points() and direct vector")
            except Exception as e2:
                logger.warning(f"query_points with direct vector failed: {str(e2)}")
        
        # Approach 3: Try query_points with dict
        if query_response is None:
            try:
                query_response = qdrant_client.query_points(
                    collection_name=collection_name,
                    query={"vector": query_embedding},
                    limit=top_k
                )
                logger.info("Query succeeded with query_points() and dict")
            except Exception as e3:
                logger.warning(f"query_points with dict failed: {str(e3)}")
        
        # If all approaches failed, return empty
        if query_response is None:
            logger.error("All query methods failed - Qdrant API may have changed")
            return []
        
        # Format results
        chunks = []
        if query_response and hasattr(query_response, 'points'):
            for point in query_response.points:
                chunk_data = {
                    "chunk_id": str(point.id),
                    "text": point.payload.get("text", "") if point.payload else "",
                    "chapter_id": point.payload.get("chapter_id", "") if point.payload else "",
                    "relevance_score": point.score if hasattr(point, 'score') else 0.0,
                    "metadata": point.payload.get("metadata", {}) if point.payload else {}
                }
                chunks.append(chunk_data)
        elif query_response:
            # If response is a list directly
            for point in query_response:
                chunk_data = {
                    "chunk_id": str(point.id),
                    "text": point.payload.get("text", "") if point.payload else "",
                    "chapter_id": point.payload.get("chapter_id", "") if point.payload else "",
                    "relevance_score": point.score if hasattr(point, 'score') else 0.0,
                    "metadata": point.payload.get("metadata", {}) if point.payload else {}
                }
                chunks.append(chunk_data)
        
        logger.info(f"Retrieved {len(chunks)} similar chunks from Qdrant")
        return chunks
    
    except Exception as e:
        logger.error(f"Error querying Qdrant: {str(e)}")
        raise


def generate_answer(question: str, context_chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Generate answer using OpenAI ChatKit SDK with retrieved context.
    
    Args:
        question: User's question
        context_chunks: List of relevant chunks from Qdrant
    
    Returns:
        Dictionary with answer text and model information
    """
    try:
        # Build context from chunks
        context = "\n\n".join([
            f"[From {chunk['chapter_id']}]: {chunk['text']}"
            for chunk in context_chunks
        ])
        
        # Create prompt
        system_prompt = """You are a helpful assistant that answers questions about the book content.
Answer questions using ONLY the provided context from the book. If the answer cannot be found in the context, 
clearly state that the information is not available in the book content. Do not use any external knowledge."""
        
        user_prompt = f"""Context from the book:
{context}

Question: {question}

Please provide an accurate answer based only on the context above."""
        
        # Call OpenAI API (using standard API, not ChatKit SDK as it may not be available)
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
        
        logger.info(f"Generated answer using {model_used}")
        
        return {
            "answer_text": answer_text,
            "model_used": model_used
        }
    
    except Exception as e:
        logger.error(f"Error generating answer: {str(e)}")
        raise


def ask_question(question: str) -> Answer:
    """
    Complete RAG pipeline: embed question, query Qdrant, generate answer.
    
    Args:
        question: User's question
    
    Returns:
        Answer object with answer text and source citations
    """
    try:
        # Step 1: Embed the question
        from src.services.ingestion_service import create_embeddings as create_embeddings_func
        query_embeddings = create_embeddings_func([question])
        query_embedding = query_embeddings[0]
        
        # Step 2: Query Qdrant for similar chunks
        similar_chunks = query_qdrant(query_embedding, top_k=5)
        
        if not similar_chunks:
            # No relevant chunks found
            return Answer(
                answer_id=str(uuid.uuid4()),
                answer_text="I couldn't find relevant information in the book to answer your question. Please try rephrasing or asking about a different topic.",
                sources=[],
                confidence=0.0,
                error_message="No relevant chunks found"
            )
        
        # Step 3: Generate answer using context
        answer_data = generate_answer(question, similar_chunks)
        
        # Step 4: Create source citations
        sources = []
        for chunk in similar_chunks:
            citation = SourceCitation(
                chunk_id=chunk["chunk_id"],
                chapter_id=chunk["chapter_id"],
                relevance_score=chunk["relevance_score"],
                snippet=chunk["text"][:200] + "..." if len(chunk["text"]) > 200 else chunk["text"]
            )
            sources.append(citation)
        
        # Step 5: Create Answer object
        answer = Answer(
            answer_id=str(uuid.uuid4()),
            answer_text=answer_data["answer_text"],
            sources=sources,
            confidence=similar_chunks[0]["relevance_score"] if similar_chunks else 0.0,
            model_used=answer_data["model_used"]
        )
        
        return answer
    
    except Exception as e:
        logger.error(f"Error in ask_question: {str(e)}")
        # Return error answer
        return Answer(
            answer_id=str(uuid.uuid4()),
            answer_text="I encountered an error while processing your question. Please try again later.",
            sources=[],
            confidence=0.0,
            error_message=str(e)
        )

