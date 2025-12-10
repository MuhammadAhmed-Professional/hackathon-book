"""
SourceCitation model for representing source references in answers.
"""
from dataclasses import dataclass


@dataclass
class SourceCitation:
    """
    Represents a citation to a source chunk used in generating an answer.
    
    Attributes:
        chunk_id: ID of the source chunk
        chapter_id: ID of the source chapter
        relevance_score: Similarity score (0.0 to 1.0)
        snippet: Relevant text snippet from the chunk
    """
    chunk_id: str
    chapter_id: str
    relevance_score: float
    snippet: str
    
    def to_dict(self) -> dict:
        """Convert citation to dictionary."""
        return {
            "chunk_id": self.chunk_id,
            "chapter_id": self.chapter_id,
            "relevance_score": self.relevance_score,
            "snippet": self.snippet
        }

