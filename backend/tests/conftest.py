"""
Shared pytest fixtures for backend testing.
"""
import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch, MagicMock
from datetime import datetime

from src.api.main import app
from src.services.auth_service import create_access_token


@pytest.fixture
def client():
    """
    FastAPI test client for making API requests.

    Returns:
        TestClient instance
    """
    return TestClient(app)


@pytest.fixture
def auth_headers():
    """
    Authenticated request headers with valid JWT token.

    Returns:
        Dict with Authorization header
    """
    token = create_access_token(user_id=1, email="test@example.com")
    return {"Authorization": f"Bearer {token}"}




@pytest.fixture
def sample_user_data():
    """
    Sample user data for testing.

    Returns:
        Dict with user information
    """
    return {
        "id": 1,
        "email": "testuser@example.com",
        "password_hash": "$2b$12$LQv3c1yqBWVHxkd0LHAkCOYz6TtxMQJqhN8/LewY5GyYqKkmpg",  # "password123"
        "software_background": {
            "python_experience": "intermediate",
            "ros_experience": "beginner",
            "cpp_experience": "none"
        },
        "hardware_background": {
            "has_jetson": False,
            "has_robot": False
        },
        "created_at": datetime(2024, 1, 1, 12, 0, 0),
        "last_login": datetime(2024, 1, 15, 10, 30, 0)
    }


@pytest.fixture
def sample_conversation_data():
    """
    Sample conversation data for testing RAG endpoints.

    Returns:
        Dict with conversation information
    """
    return {
        "id": 1,
        "user_id": 1,
        "session_id": "test-session-123",
        "question": "What is ROS 2?",
        "answer": "ROS 2 is the Robot Operating System version 2, a modern robotics middleware.",
        "question_type": "rag",
        "created_at": datetime(2024, 1, 15, 10, 30, 0)
    }


@pytest.fixture(autouse=True)
def mock_env_variables(monkeypatch):
    """
    Set required environment variables for testing.
    """
    monkeypatch.setenv("BETTER_AUTH_SECRET", "test-secret-key-for-jwt-tokens")
    monkeypatch.setenv("OPENAI_API_KEY", "test-openai-key")
    monkeypatch.setenv("QDRANT_URL", "http://localhost:6333")
    monkeypatch.setenv("QDRANT_API_KEY", "test-qdrant-key")
    monkeypatch.setenv("DATABASE_URL", "postgresql://test:test@localhost/test")
