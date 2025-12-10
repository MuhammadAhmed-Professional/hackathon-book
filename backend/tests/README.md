# Backend Test Suite

Comprehensive pytest test suite for the Physical AI Textbook backend API.

## Test Coverage

### Test Files

1. **`conftest.py`** - Shared fixtures and test configuration
   - `client`: FastAPI TestClient
   - `auth_headers`: Authenticated request headers with JWT
   - `mock_openai`: Mock OpenAI API calls
   - `mock_qdrant`: Mock Qdrant vector database
   - `sample_user_data`: Sample user data for testing
   - `sample_conversation_data`: Sample conversation data

2. **`test_auth.py`** - Authentication endpoint tests (15 tests)
   - Signup with valid/invalid data
   - Login with correct/incorrect credentials
   - User profile retrieval
   - JWT token validation
   - Error handling for auth flows

3. **`test_ask.py`** - RAG chatbot endpoint tests (10 tests)
   - Question answering with RAG
   - Source citation inclusion
   - Conversation persistence
   - Personalization context integration
   - Session ID handling
   - Error scenarios

4. **`test_skills.py`** - Agent skills endpoint tests (15 tests)
   - List available skills
   - SummarizeSection skill execution
   - GenerateQuizQuestions skill execution
   - ExplainTerm skill execution
   - Skill parameter validation
   - Error handling

5. **`test_e2e.py`** - End-to-end integration tests (4 test suites)
   - Complete user journey (signup → login → ask)
   - Agent skills journey (all 3 skills)
   - Personalization journey
   - Error handling across endpoints

## Running Tests

### Install dependencies
```bash
pip install -r requirements-test.txt
```

### Run all tests
```bash
pytest
```

### Run with coverage
```bash
pytest --cov=src --cov-report=html
```

### Run specific test file
```bash
pytest tests/test_auth.py
```

### Run integration tests only
```bash
pytest -m integration
```

### Run with verbose output
```bash
pytest -v
```

## Test Statistics

- **Total Test Files**: 5
- **Total Tests**: 44+
- **Coverage Target**: 70% minimum
- **Endpoints Covered**: 13/13 (100%)

### Endpoint Coverage

| Endpoint | Tests | Coverage |
|----------|-------|----------|
| POST /auth/signup | 5 | 100% |
| POST /auth/signin | 4 | 100% |
| GET /auth/me | 4 | 100% |
| POST /ask | 10 | 100% |
| GET /skills | 1 | 100% |
| POST /skills/summarize | 4 | 100% |
| POST /skills/quiz | 4 | 100% |
| POST /skills/explain | 5 | 100% |
| Integration flows | 4 | 100% |

## Test Principles

All tests follow these principles from `write-comprehensive-tests.md` skill:

- **AAA Pattern**: Arrange-Act-Assert for clarity
- **Isolation**: Each test is independent with mocked dependencies
- **Clear Names**: Descriptive test names explain what is tested
- **Fast**: All tests run in <1 second using mocks
- **Comprehensive**: Happy paths + error paths + edge cases
- **Realistic Data**: Meaningful test data that reflects real usage

## Mocking Strategy

- **OpenAI API**: Mocked to avoid real API calls and costs
- **Qdrant**: Mocked to avoid vector database dependency
- **Database**: Mocked with `patch` for service layer functions
- **Authentication**: Real JWT token generation, mocked user data

## CI/CD Integration

Tests are configured for CI/CD with:
- Coverage reporting (HTML + terminal)
- Minimum coverage threshold (70%)
- Pytest markers for test categorization
- Exit code on test failure

## Next Steps

1. Run tests locally: `pytest`
2. Review coverage report: `open htmlcov/index.html`
3. Add tests for new features as they're developed
4. Maintain >70% code coverage for production deployment
