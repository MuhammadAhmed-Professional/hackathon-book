"""
Answer model for representing chatbot responses.
"""
from typing import Optional, List
from dataclasses import dataclass
from datetime import datetime

from .source_citation import SourceCitation


@dataclass
class Answer:
    """
    Represents the chatbot's answer to a question.
    
    Attributes:
        answer_id: Unique identifier for the answer
        answer_text: The actual answer text
        sources: List of source citations (chunk references)
        confidence: Confidence score (0.0 to 1.0)
        question_id: ID of the question this answers
        timestamp: When the answer was generated
        model_used: Which AI model was used (e.g., 'gpt-3.5-turbo')
        error_message: Error message if generation failed
    """
    answer_id: str
    answer_text: str
    sources: List[SourceCitation]
    confidence: float = 1.0
    question_id: Optional[str] = None
    timestamp: Optional[datetime] = None
    model_used: Optional[str] = None
    error_message: Optional[str] = None
    
    def __post_init__(self):
        """Set timestamp if not provided."""
        if self.timestamp is None:
            self.timestamp = datetime.utcnow()
    
    def to_dict(self) -> dict:
        """Convert answer to dictionary for API response."""
        return {
            "answer_id": self.answer_id,
            "answer_text": self.answer_text,
            "sources": [source.to_dict() for source in self.sources],
            "confidence": self.confidence,
            "question_id": self.question_id,
            "timestamp": self.timestamp.isoformat() if self.timestamp else None,
            "model_used": self.model_used,
            "error_message": self.error_message
        }

