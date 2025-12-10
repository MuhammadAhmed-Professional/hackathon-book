"""
Question model for representing user questions.
"""
from typing import Optional
from dataclasses import dataclass
from datetime import datetime


@dataclass
class Question:
    """
    Represents a user's question about the book.
    
    Attributes:
        question_id: Unique identifier for the question
        question_text: The actual question text
        mode: Question mode ('rag' or 'selected_text')
        timestamp: When the question was asked
        session_id: Optional session identifier for conversation tracking
    """
    question_id: str
    question_text: str
    mode: str = "rag"  # 'rag' or 'selected_text'
    timestamp: Optional[datetime] = None
    session_id: Optional[str] = None
    
    def __post_init__(self):
        """Set timestamp if not provided."""
        if self.timestamp is None:
            self.timestamp = datetime.utcnow()
    
    def to_dict(self) -> dict:
        """Convert question to dictionary."""
        return {
            "question_id": self.question_id,
            "question_text": self.question_text,
            "mode": self.mode,
            "timestamp": self.timestamp.isoformat() if self.timestamp else None,
            "session_id": self.session_id
        }

