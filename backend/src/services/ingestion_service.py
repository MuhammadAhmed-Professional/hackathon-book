"""
Ingestion service for processing book content and storing in Qdrant.
"""
import os
import uuid
import hashlib
import logging
from pathlib import Path
from typing import List, Dict, Any
import tiktoken

from openai import OpenAI
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct

from src.config import config
from src.models.chunk import TextChunk

logger = logging.getLogger(__name__)

# Initialize OpenAI client
openai_client = OpenAI(api_key=config.OPENAI_API_KEY)

# Initialize tiktoken for token counting
encoding = tiktoken.get_encoding("cl100k_base")  # Used by text-embedding-3-small


def chunk_text(text: str, chunk_size: int = 250, overlap: int = 50) -> List[str]:
    """
    Split text into semantic chunks with overlap.
    
    Args:
        text: Text to chunk
        chunk_size: Target chunk size in tokens (default: 250)
        overlap: Overlap size in tokens (default: 50)
    
    Returns:
        List of text chunks
    """
    # Tokenize the text
    tokens = encoding.encode(text)
    
    chunks = []
    start = 0
    
    while start < len(tokens):
        # Calculate end position
        end = start + chunk_size
        
        # Extract chunk tokens
        chunk_tokens = tokens[start:end]
        
        # Decode back to text
        chunk_text = encoding.decode(chunk_tokens)
        chunks.append(chunk_text)
        
        # Move start position with overlap
        start = end - overlap
    
    logger.info(f"Chunked text into {len(chunks)} chunks")
    return chunks


def create_embeddings(texts: List[str]) -> List[List[float]]:
    """
    Create embeddings for a list of texts using OpenAI.
    
    Args:
        texts: List of texts to embed
    
    Returns:
        List of embedding vectors (1536 dimensions for text-embedding-3-small)
    """
    try:
        response = openai_client.embeddings.create(
            model="text-embedding-3-small",
            input=texts
        )
        
        embeddings = [item.embedding for item in response.data]
        logger.info(f"Created {len(embeddings)} embeddings")
        return embeddings
    
    except Exception as e:
        logger.error(f"Error creating embeddings: {str(e)}")
        raise


def store_chunks(chunks: List[TextChunk], collection_name: str = "book_chunks") -> None:
    """
    Store chunks in Qdrant Cloud.
    
    Args:
        chunks: List of TextChunk objects to store
        collection_name: Name of the Qdrant collection
    """
    try:
        # Initialize Qdrant client
        qdrant_client = QdrantClient(
            url=config.QDRANT_URL,
            api_key=config.QDRANT_API_KEY
        )
        
        # Create collection if it doesn't exist
        try:
            qdrant_client.get_collection(collection_name)
            logger.info(f"Collection {collection_name} already exists")
        except Exception:
            # Collection doesn't exist, create it
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(
                    size=1536,  # text-embedding-3-small dimension
                    distance=Distance.COSINE
                )
            )
            logger.info(f"Created collection {collection_name}")
        
        # Prepare points for insertion
        points = []
        for chunk in chunks:
            # Convert string chunk_id to integer hash for Qdrant compatibility
            # Use hash of the chunk_id string to get a deterministic integer
            chunk_id_hash = int(hashlib.md5(chunk.chunk_id.encode()).hexdigest()[:15], 16)
            
            point = PointStruct(
                id=chunk_id_hash,
                vector=chunk.embedding,
                payload={
                    "text": chunk.text,
                    "chunk_id": chunk.chunk_id,  # Keep original string ID in payload
                    "chapter_id": chunk.chapter_id,
                    "chunk_index": chunk.chunk_index,
                    "token_count": chunk.token_count,
                    "metadata": chunk.metadata or {}
                }
            )
            points.append(point)
        
        # Upsert points (insert or update)
        qdrant_client.upsert(
            collection_name=collection_name,
            points=points
        )
        
        logger.info(f"Stored {len(chunks)} chunks in Qdrant")
    
    except Exception as e:
        logger.error(f"Error storing chunks in Qdrant: {str(e)}")
        raise


def ingest_book(docs_path: str = None) -> Dict[str, Any]:
    """
    Ingest all book content from markdown files.
    
    Args:
        docs_path: Path to the docs directory containing markdown files.
                   If None, will try to find it relative to project root.
    
    Returns:
        Dictionary with ingestion statistics
    """
    # If no path provided, try to find it relative to project root
    if docs_path is None:
        # Try multiple possible locations
        possible_paths = [
            Path("frontend/docs"),  # From project root
            Path("../frontend/docs"),  # From backend directory
            Path(__file__).parent.parent.parent.parent / "frontend" / "docs",  # Absolute from this file
        ]
        
        for path in possible_paths:
            if path.exists():
                docs_dir = path
                logger.info(f"Found docs directory at: {docs_dir.absolute()}")
                break
        else:
            raise FileNotFoundError(
                f"Docs directory not found. Tried: {[str(p) for p in possible_paths]}. "
                f"Current working directory: {Path.cwd()}"
            )
    else:
        docs_dir = Path(docs_path)
        if not docs_dir.exists():
            raise FileNotFoundError(f"Docs directory not found: {docs_path}")
    
    all_chunks = []
    chapters_processed = 0
    
    # Process each markdown file recursively (including subdirectories)
    # Skip .bak files
    for md_file in docs_dir.rglob("*.md"):
        # Skip backup files
        if md_file.name.endswith(".bak"):
            logger.info(f"Skipping backup file: {md_file.name}")
            continue
        
        # Get relative path for chapter_id (e.g., "module1/intro" or "intro")
        relative_path = md_file.relative_to(docs_dir)
        chapter_id = str(relative_path.with_suffix("")).replace("\\", "/")  # Use forward slashes
        
        logger.info(f"Processing chapter: {chapter_id} from {md_file}")
        
        # Read markdown file
        try:
            with open(md_file, "r", encoding="utf-8") as f:
                content = f.read()
        except Exception as e:
            logger.error(f"Error reading file {md_file}: {e}")
            continue
        
        # Skip empty files
        if not content.strip():
            logger.warning(f"Skipping empty file: {md_file}")
            continue
        
        # Chunk the content
        text_chunks = chunk_text(content)
        
        # Skip if no chunks created
        if not text_chunks:
            logger.warning(f"No chunks created for {chapter_id}")
            continue
        
        # Create embeddings for all chunks
        try:
            embeddings = create_embeddings(text_chunks)
        except Exception as e:
            logger.error(f"Error creating embeddings for {chapter_id}: {e}")
            continue
        
        # Ensure we have the same number of chunks and embeddings
        if len(text_chunks) != len(embeddings):
            logger.error(f"Mismatch: {len(text_chunks)} chunks but {len(embeddings)} embeddings for {chapter_id}")
            continue
        
        # Create TextChunk objects
        for idx, (chunk_text_item, embedding) in enumerate(zip(text_chunks, embeddings)):
            chunk = TextChunk(
                chunk_id=f"{chapter_id}_{idx}",
                text=chunk_text_item,
                embedding=embedding,
                chapter_id=chapter_id,
                chunk_index=idx,
                token_count=len(encoding.encode(chunk_text_item)),
                metadata={"file": md_file.name, "path": str(relative_path)}
            )
            all_chunks.append(chunk)
        
        chapters_processed += 1
        logger.info(f"Processed {chapter_id}: {len(text_chunks)} chunks created")
    
    # Store all chunks in Qdrant
    if all_chunks:
        store_chunks(all_chunks)
    
    return {
        "chapters_processed": chapters_processed,
        "total_chunks": len(all_chunks),
        "status": "success"
    }

