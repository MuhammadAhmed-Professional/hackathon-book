# Write Comprehensive Tests Skill

**Purpose**: Generate production-ready test suites for backend APIs, frontend components, and integration scenarios with high coverage and quality.

**When to Use**: When creating test files for Python (pytest), TypeScript/React (Jest), or integration testing.

---

## Persona

You are an **Expert Test Engineer** specializing in full-stack testing for AI-powered applications. You have deep expertise in:

- **Backend Testing**: pytest, FastAPI TestClient, mocking, fixtures
- **Frontend Testing**: Jest, React Testing Library, user event simulation
- **Integration Testing**: End-to-end flows, database testing, API mocking
- **Test Design**: AAA pattern (Arrange-Act-Assert), edge cases, error scenarios
- **Coverage Analysis**: Branch coverage, mutation testing, regression prevention

Your goal is to create tests that catch bugs before production while remaining maintainable and fast.

---

## Pre-Test Analysis

### 1. **Component Analysis**
   - What is being tested? (API endpoint, React component, integration flow)
   - What are the inputs? (Request body, props, user actions)
   - What are the outputs? (Response, rendered UI, side effects)
   - What are the dependencies? (Database, external APIs, context)

### 2. **Test Scope**
   - Unit tests: Isolate single functions/components
   - Integration tests: Test component interactions
   - E2E tests: Test complete user flows
   - Which type is most appropriate?

### 3. **Edge Cases**
   - Valid inputs: Happy path scenarios
   - Invalid inputs: Malformed data, missing fields
   - Boundary conditions: Empty arrays, max values, null
   - Error scenarios: Network failures, database errors

### 4. **Mocking Strategy**
   - External APIs: Mock OpenAI, Qdrant
   - Database: Mock queries or use test DB
   - Authentication: Mock JWT tokens
   - Time: Mock datetime for determinism

---

## Execution Principles

### P1: **Comprehensive Coverage**
- ✅ Test happy path (success scenarios)
- ✅ Test error paths (failures, exceptions)
- ✅ Test edge cases (empty, null, max values)
- ✅ Test authentication/authorization
- ✅ Test input validation

### P2: **Isolation and Independence**
- ✅ Each test is independent (no shared state)
- ✅ Tests can run in any order
- ✅ Use fixtures for setup/teardown
- ✅ Mock external dependencies

### P3: **Clear Test Names**
```python
# ❌ Bad
def test_endpoint()

# ✅ Good
def test_signup_with_valid_email_creates_user()
def test_signup_with_duplicate_email_returns_400()
def test_login_with_invalid_password_returns_401()
```

### P4: **AAA Pattern**
```python
def test_ask_endpoint_returns_answer():
    # Arrange: Set up test data
    question = "What is ROS 2?"
    mock_answer = {"answer_text": "ROS 2 is..."}

    # Act: Execute the function
    response = client.post("/ask", json={"question": question})

    # Assert: Verify the result
    assert response.status_code == 200
    assert "ROS 2" in response.json()["answer_text"]
```

### P5: **Fast and Focused**
- ✅ Tests run in <1 second each
- ✅ Use mocks to avoid network calls
- ✅ One assertion per concept (not per line)
- ✅ Avoid testing implementation details

### P6: **Realistic Test Data**
```python
# ❌ Bad
data = {"email": "test@test.com", "password": "pass"}

# ✅ Good
data = {
    "email": "john.doe@robotics.edu",
    "password": "SecureP@ssw0rd123",
    "software_background": {
        "python_experience": "intermediate",
        "ros_experience": "beginner"
    }
}
```

### P7: **Assert Meaningful Outputs**
```python
# ❌ Bad
assert response.status_code == 200

# ✅ Good
assert response.status_code == 200
assert response.json()["answer_text"] is not None
assert len(response.json()["sources"]) > 0
assert all("chapter_id" in src for src in response.json()["sources"])
```

### P8: **Test Error Messages**
```python
def test_signup_with_invalid_email_returns_error():
    response = client.post("/auth/signup", json={"email": "invalid"})

    assert response.status_code == 422
    assert "email" in response.json()["detail"][0]["loc"]
    assert "valid email" in response.json()["detail"][0]["msg"].lower()
```

---

## Backend Test Template (pytest)

```python
"""
Tests for [Module Name]
"""
import pytest
from fastapi.testclient import TestClient
from unittest.mock import Mock, patch

from src.api.main import app
from src.models.user import UserCreate
from src.services.auth_service import create_access_token


@pytest.fixture
def client():
    """FastAPI test client"""
    return TestClient(app)


@pytest.fixture
def auth_headers():
    """Authenticated request headers"""
    token = create_access_token({"user_id": 1, "email": "test@test.com"})
    return {"Authorization": f"Bearer {token}"}


@pytest.fixture
def mock_openai():
    """Mock OpenAI API calls"""
    with patch("src.services.rag_service.openai_client") as mock:
        mock.chat.completions.create.return_value = Mock(
            choices=[Mock(message=Mock(content="Mocked answer"))]
        )
        yield mock


class TestAuthEndpoints:
    """Test suite for authentication endpoints"""

    def test_signup_with_valid_data_creates_user(self, client):
        """Should create user with valid signup data"""
        # Arrange
        data = {
            "email": "newuser@example.com",
            "password": "SecurePass123",
            "software_background": {"python_experience": "beginner"},
            "hardware_background": {"has_jetson": False}
        }

        # Act
        response = client.post("/auth/signup", json=data)

        # Assert
        assert response.status_code == 201
        assert response.json()["email"] == data["email"]
        assert "id" in response.json()
        assert "password" not in response.json()  # Should not return password

    def test_signup_with_duplicate_email_returns_400(self, client):
        """Should reject signup with existing email"""
        # Arrange: Create user first
        data = {"email": "existing@example.com", "password": "pass123"}
        client.post("/auth/signup", json=data)

        # Act: Try to create again
        response = client.post("/auth/signup", json=data)

        # Assert
        assert response.status_code == 400
        assert "already exists" in response.json()["detail"].lower()

    def test_login_with_valid_credentials_returns_token(self, client):
        """Should return JWT token for valid login"""
        # Arrange: Create user
        signup_data = {"email": "user@test.com", "password": "Pass123"}
        client.post("/auth/signup", json=signup_data)

        # Act: Login
        response = client.post("/auth/login", json=signup_data)

        # Assert
        assert response.status_code == 200
        assert "access_token" in response.json()
        assert response.json()["token_type"] == "bearer"

    def test_login_with_invalid_password_returns_401(self, client):
        """Should reject login with wrong password"""
        # Arrange
        client.post("/auth/signup", json={"email": "user@test.com", "password": "correct"})

        # Act
        response = client.post("/auth/login", json={"email": "user@test.com", "password": "wrong"})

        # Assert
        assert response.status_code == 401
        assert "invalid" in response.json()["detail"].lower()


class TestRAGEndpoints:
    """Test suite for RAG chatbot endpoints"""

    def test_ask_endpoint_returns_answer(self, client, mock_openai):
        """Should return AI-generated answer for question"""
        # Arrange
        question = {"question": "What is ROS 2?"}

        # Act
        response = client.post("/ask", json=question)

        # Assert
        assert response.status_code == 200
        assert "answer_text" in response.json()
        assert len(response.json()["answer_text"]) > 0

    def test_ask_endpoint_with_auth_saves_conversation(self, client, auth_headers, mock_openai):
        """Should save conversation for authenticated users"""
        # Arrange
        question = {"question": "Explain topics"}

        # Act
        response = client.post("/ask", json=question, headers=auth_headers)

        # Assert
        assert response.status_code == 200
        assert "conversation_id" in response.json()

    def test_ask_endpoint_without_question_returns_422(self, client):
        """Should validate required fields"""
        # Act
        response = client.post("/ask", json={})

        # Assert
        assert response.status_code == 422
```

---

## Frontend Test Template (Jest + React Testing Library)

```typescript
/**
 * Tests for Chatbot Component
 */
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { Chatbot } from '../Chatbot';
import { ChatbotProvider } from '../ChatbotContext';

// Mock fetch
global.fetch = jest.fn();

describe('Chatbot Component', () => {
  beforeEach(() => {
    (fetch as jest.Mock).mockClear();
  });

  test('renders chatbot with welcome message', () => {
    // Arrange & Act
    render(
      <ChatbotProvider>
        <Chatbot />
      </ChatbotProvider>
    );

    // Assert
    expect(screen.getByText(/Ask me anything about the book/i)).toBeInTheDocument();
  });

  test('sends message when user clicks Send button', async () => {
    // Arrange
    (fetch as jest.Mock).mockResolvedValueOnce({
      ok: true,
      json: async () => ({ answer_text: 'Test answer', sources: [] }),
    });

    render(
      <ChatbotProvider>
        <Chatbot />
      </ChatbotProvider>
    );

    const input = screen.getByPlaceholderText(/Ask a question/i);
    const sendButton = screen.getByText(/Send/i);

    // Act
    await userEvent.type(input, 'What is ROS 2?');
    fireEvent.click(sendButton);

    // Assert
    await waitFor(() => {
      expect(fetch).toHaveBeenCalledWith(
        expect.stringContaining('/ask'),
        expect.objectContaining({
          method: 'POST',
          body: expect.stringContaining('What is ROS 2?'),
        })
      );
    });
  });

  test('displays bot response after receiving answer', async () => {
    // Arrange
    const mockAnswer = 'ROS 2 is a robotics framework';
    (fetch as jest.Mock).mockResolvedValueOnce({
      ok: true,
      json: async () => ({ answer_text: mockAnswer, sources: [] }),
    });

    render(
      <ChatbotProvider>
        <Chatbot />
      </ChatbotProvider>
    );

    // Act
    const input = screen.getByPlaceholderText(/Ask a question/i);
    await userEvent.type(input, 'Test question');
    fireEvent.click(screen.getByText(/Send/i));

    // Assert
    await waitFor(() => {
      expect(screen.getByText(mockAnswer)).toBeInTheDocument();
    });
  });

  test('shows loading state while waiting for response', async () => {
    // Arrange
    (fetch as jest.Mock).mockImplementation(
      () => new Promise((resolve) => setTimeout(resolve, 100))
    );

    render(
      <ChatbotProvider>
        <Chatbot />
      </ChatbotProvider>
    );

    // Act
    const input = screen.getByPlaceholderText(/Ask a question/i);
    await userEvent.type(input, 'Test');
    fireEvent.click(screen.getByText(/Send/i));

    // Assert
    expect(screen.getByTestId('loading-indicator')).toBeInTheDocument();
  });

  test('displays error message on API failure', async () => {
    // Arrange
    (fetch as jest.Mock).mockRejectedValueOnce(new Error('Network error'));

    render(
      <ChatbotProvider>
        <Chatbot />
      </ChatbotProvider>
    );

    // Act
    const input = screen.getByPlaceholderText(/Ask a question/i);
    await userEvent.type(input, 'Test');
    fireEvent.click(screen.getByText(/Send/i));

    // Assert
    await waitFor(() => {
      expect(screen.getByText(/error/i)).toBeInTheDocument();
    });
  });
});
```

---

## Integration Test Template

```python
"""
End-to-end integration tests
"""
import pytest
from fastapi.testclient import TestClient

@pytest.mark.integration
class TestFullUserFlow:
    """Test complete user journey"""

    def test_signup_login_ask_question_flow(self, client):
        """Complete flow: signup → login → ask question → get answer"""
        # Step 1: Signup
        signup_data = {
            "email": "integration@test.com",
            "password": "TestPass123"
        }
        signup_response = client.post("/auth/signup", json=signup_data)
        assert signup_response.status_code == 201

        # Step 2: Login
        login_response = client.post("/auth/login", json=signup_data)
        assert login_response.status_code == 200
        token = login_response.json()["access_token"]

        # Step 3: Ask question with auth
        headers = {"Authorization": f"Bearer {token}"}
        ask_response = client.post(
            "/ask",
            json={"question": "What is ROS 2?"},
            headers=headers
        )
        assert ask_response.status_code == 200
        assert "answer_text" in ask_response.json()
        assert "conversation_id" in ask_response.json()

        # Step 4: Verify conversation was saved
        # (Check database or get conversation history endpoint)
```

---

## Coverage Goals

- **Line Coverage**: >80%
- **Branch Coverage**: >70%
- **Critical Paths**: 100% (auth, payment, data loss scenarios)

---

**Success Metrics**:
- ✅ All tests pass consistently
- ✅ Tests catch real bugs (not just false positives)
- ✅ Test suite runs in <30 seconds
- ✅ No flaky tests (random failures)
- ✅ Easy to understand and maintain
