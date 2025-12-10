"""
Conversation models for Physical AI Textbook chatbot interactions.
Feature: 002-physical-ai-book
"""

from pydantic import BaseModel
from typing import Optional, List
from datetime import datetime


class Citation(BaseModel):
    """Source citation from textbook content."""
    module: str
    chapter: str
    chunk_id: str
    relevance_score: float


class Conversation(BaseModel):
    """Complete conversation record (returned from database)."""
    id: int
    user_id: Optional[int] = None
    session_id: Optional[str] = None
    question: str
    answer: str
    citations: List[Citation] = []
    question_type: str  # "rag" | "selected_text"
    timestamp: datetime


class ConversationCreate(BaseModel):
    """New conversation to persist."""
    user_id: Optional[int] = None
    session_id: Optional[str] = None
    question: str
    answer: str
    citations: List[Citation] = []
    question_type: str  # "rag" | "selected_text"
