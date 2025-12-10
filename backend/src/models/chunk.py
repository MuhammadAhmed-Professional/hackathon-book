"""
TextChunk model for representing book content chunks stored in vector database.
"""
from typing import Optional, Dict, Any
from dataclasses import dataclass


@dataclass
class TextChunk:
    """
    Represents a chunk of text from the book with its embedding and metadata.
    
    Attributes:
        chunk_id: Unique identifier for the chunk
        text: The actual text content of the chunk
        embedding: Vector embedding of the text (list of floats)
        chapter_id: Identifier of the source chapter
        chunk_index: Index of this chunk within the chapter
        token_count: Number of tokens in the chunk
        metadata: Additional metadata (e.g., section title, page number)
    """
    chunk_id: str
    text: str
    embedding: list[float]
    chapter_id: str
    chunk_index: int
    token_count: int
    metadata: Optional[Dict[str, Any]] = None
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert chunk to dictionary for storage."""
        return {
            "chunk_id": self.chunk_id,
            "text": self.text,
            "embedding": self.embedding,
            "chapter_id": self.chapter_id,
            "chunk_index": self.chunk_index,
            "token_count": self.token_count,
            "metadata": self.metadata or {}
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TextChunk":
        """Create chunk from dictionary."""
        return cls(
            chunk_id=data["chunk_id"],
            text=data["text"],
            embedding=data["embedding"],
            chapter_id=data["chapter_id"],
            chunk_index=data["chunk_index"],
            token_count=data["token_count"],
            metadata=data.get("metadata")
        )

