"""
Basic API tests - simplified to test critical endpoints without deep mocking.

These tests verify that endpoints are accessible and return proper responses.
"""
import pytest
from unittest.mock import patch, Mock


class TestHealthEndpoints:
    """Test health check endpoints."""

    def test_root_endpoint_returns_200(self, client):
        """Root endpoint should return status."""
        response = client.get("/")
        assert response.status_code == 200
        assert "status" in response.json()

    def test_ping_endpoint_returns_200(self, client):
        """Ping endpoint should return OK."""
        response = client.get("/ping")
        assert response.status_code == 200


class TestAuthEndpointsBasic:
    """Basic authentication endpoint tests."""

    def test_signup_requires_all_fields(self, client):
        """Signup should validate required fields."""
        # Missing fields
        response = client.post("/auth/signup", json={})
        assert response.status_code == 422

    def test_signup_validates_email_format(self, client):
        """Signup should validate email format."""
        data = {
            "email": "invalid-email",
            "password": "password123",
            "software_background": {"python_experience": "beginner"},
            "hardware_background": {"has_jetson": False}
        }
        response = client.post("/auth/signup", json=data)
        assert response.status_code == 422

    def test_signin_requires_credentials(self, client):
        """Signin should require email and password."""
        response = client.post("/auth/signin", json={})
        assert response.status_code == 422

    def test_get_me_without_token_returns_401(self, client):
        """GET /auth/me should require authentication."""
        response = client.get("/auth/me")
        assert response.status_code == 401


class TestSkillsEndpointsBasic:
    """Basic skills endpoint tests."""

    def test_list_skills_returns_available_skills(self, client):
        """GET /skills should list all available skills."""
        response = client.get("/skills")
        assert response.status_code == 200
        data = response.json()
        assert "skills" in data
        assert len(data["skills"]) > 0

    def test_summarize_requires_content(self, client):
        """Summarize skill should require content field."""
        response = client.post("/skills/summarize", json={})
        assert response.status_code == 422

    def test_quiz_requires_content(self, client):
        """Quiz skill should require content field."""
        response = client.post("/skills/quiz", json={})
        assert response.status_code == 422

    def test_explain_requires_term(self, client):
        """Explain skill should require term field."""
        response = client.post("/skills/explain", json={})
        assert response.status_code == 422


class TestAskEndpointBasic:
    """Basic RAG endpoint tests."""

    def test_ask_requires_question(self, client):
        """Ask endpoint should require question field."""
        response = client.post("/ask", json={})
        assert response.status_code == 422

    def test_ask_rejects_empty_question(self, client):
        """Ask endpoint should reject empty questions."""
        response = client.post("/ask", json={"question": ""})
        assert response.status_code == 400


