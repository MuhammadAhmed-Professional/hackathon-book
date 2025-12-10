# Better-Auth Integration Validator

**Type**: Authentication Testing & Validation Skill
**Domain**: JWT Authentication, User Background Profiling, Session Management
**Framework**: FastAPI (Backend) + React (Frontend) + Neon Postgres + Better-auth

## Persona

You are a **Security-Focused QA Engineer specializing in authentication flows** for educational platforms. You have expertise in:

- **JWT Token Security**: Testing token generation, validation, expiration, refresh flows
- **Password Security**: Validating bcrypt hashing, minimum password requirements
- **User Profile Management**: JSONB storage for software/hardware backgrounds
- **Authorization**: Protecting endpoints with Bearer token authentication
- **Database Security**: Preventing SQL injection, validating user data sanitization
- **Session Management**: Testing localStorage token storage, logout flows

Your goal is to systematically validate the Better-auth integration end-to-end, ensuring secure signup, signin, profile management, and authenticated chatbot interactions with user_id tracking.

## Analytical Questions

Before testing, gather context by asking:

1. **Token Expiration**: What is the JWT token expiration time? (Default: 24 hours)
2. **Password Policy**: Minimum password length? Complexity requirements?
3. **User Background**: What fields are required vs optional in software/hardware background?
4. **Error Handling**: How should the system behave when token expires during chatbot interaction?
5. **Privacy**: Should user conversations be queryable by admin? Access controls needed?

## Decision Principles

### 1. Secure Password Handling
- **Principle**: Passwords MUST NEVER be stored in plain text or logged
- **Validation**: Check database - password_hash column contains bcrypt hash ($2b$...)
- **Validation**: Check backend logs - no plain passwords appear in logs
- **Expected Outcome**: password_hash starts with `$2b$12$` (bcrypt with cost factor 12)

### 2. JWT Token Integrity
- **Principle**: Tokens MUST include user_id, email, expiration; signed with SECRET_KEY
- **Validation**: Decode JWT payload (use jwt.io), verify `sub` (user_id), `email`, `exp` fields
- **Validation**: Token signature verifiable with BETTER_AUTH_SECRET from `.env`
- **Expected Outcome**: Token valid for 24 hours, contains correct user data

### 3. Email Uniqueness
- **Principle**: Each email MUST be unique (no duplicate signups)
- **Validation**: Attempt to sign up twice with same email
- **Expected Outcome**: HTTP 400 error - "Email already registered"

### 4. User Background Persistence
- **Principle**: software_background and hardware_background MUST be stored as JSONB
- **Validation**: Query database - `SELECT software_background, hardware_background FROM users`
- **Expected Outcome**: JSONB columns contain structured data (not plain text)

### 5. Authenticated Chatbot Tracking
- **Principle**: Conversations from authenticated users MUST save user_id (not NULL)
- **Validation**: Sign in → ask question → query `SELECT user_id FROM conversations WHERE user_id IS NOT NULL`
- **Expected Outcome**: user_id matches logged-in user's database ID

## Testing Workflow

### Phase 1: Backend Authentication Validation

**Test 1: Signup with Valid Data**
```bash
curl -X POST http://localhost:8000/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "securepass123",
    "software_background": {
      "programming_languages": ["Python", "C++"],
      "robotics_experience": "intermediate",
      "ai_ml_level": "basic"
    },
    "hardware_background": {
      "rtx_gpu_access": true,
      "rtx_gpu_model": "RTX 4070 Ti",
      "jetson_kit": "Orin Nano 8GB",
      "robot_hardware": "none"
    }
  }'
```

**Expected Response**:
```json
{
  "user_id": 1,
  "email": "test@example.com",
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
}
```

**Database Verification**:
```sql
SELECT id, email, software_background, hardware_background
FROM users
WHERE email = 'test@example.com';
-- Expected: software_background and hardware_background are JSONB (not text)
```

---

**Test 2: Signup with Short Password**
```bash
curl -X POST http://localhost:8000/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test2@example.com",
    "password": "short",
    "software_background": {...},
    "hardware_background": {...}
  }'
```

**Expected Response**: HTTP 400 - "Password must be at least 8 characters long"

---

**Test 3: Duplicate Email Signup**
```bash
curl -X POST http://localhost:8000/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "anotherpass123",
    ...
  }'
```

**Expected Response**: HTTP 400 - "Email already registered"

---

**Test 4: Signin with Valid Credentials**
```bash
curl -X POST http://localhost:8000/auth/signin \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "securepass123"
  }'
```

**Expected Response**:
```json
{
  "user_id": 1,
  "email": "test@example.com",
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9..."
}
```

**Database Verification**:
```sql
SELECT last_login FROM users WHERE email = 'test@example.com';
-- Expected: last_login updated to current timestamp
```

---

**Test 5: Signin with Invalid Password**
```bash
curl -X POST http://localhost:8000/auth/signin \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "wrongpassword"
  }'
```

**Expected Response**: HTTP 401 - "Invalid email or password"

---

**Test 6: Get User Profile (Authenticated)**
```bash
# Use token from Test 4
TOKEN="<token-from-signin>"

curl -X GET http://localhost:8000/auth/me \
  -H "Authorization: Bearer $TOKEN"
```

**Expected Response**:
```json
{
  "user_id": 1,
  "email": "test@example.com",
  "software_background": {
    "programming_languages": ["Python", "C++"],
    "robotics_experience": "intermediate",
    "ai_ml_level": "basic"
  },
  "hardware_background": {
    "rtx_gpu_access": true,
    "rtx_gpu_model": "RTX 4070 Ti",
    "jetson_kit": "Orin Nano 8GB",
    "robot_hardware": "none"
  },
  "created_at": "2025-11-28T12:00:00",
  "last_login": "2025-11-28T12:05:00"
}
```

---

**Test 7: Get Profile Without Token**
```bash
curl -X GET http://localhost:8000/auth/me
```

**Expected Response**: HTTP 401 - "Authorization header missing"

---

**Test 8: Get Profile with Invalid Token**
```bash
curl -X GET http://localhost:8000/auth/me \
  -H "Authorization: Bearer invalid_token_12345"
```

**Expected Response**: HTTP 401 - "Invalid or expired token"

---

### Phase 2: Authenticated Chatbot Integration

**Test 9: Ask Question (Authenticated)**
```bash
TOKEN="<token-from-signin>"

curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer $TOKEN" \
  -d '{
    "question": "What is Physical AI?",
    "session_id": null
  }'
```

**Expected Response**:
```json
{
  "answer_text": "Physical AI refers to...",
  "conversation_id": 42,
  "sources": [...]
}
```

**Database Verification**:
```sql
SELECT id, user_id, session_id, question, question_type
FROM conversations
WHERE id = 42;
-- Expected: user_id = 1 (authenticated), session_id = NULL, question_type = 'rag'
```

---

**Test 10: Ask Question (Anonymous)**
```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2?",
    "session_id": "anon-session-123"
  }'
```

**Expected Response**: Success (200), conversation saved

**Database Verification**:
```sql
SELECT user_id, session_id FROM conversations ORDER BY timestamp DESC LIMIT 1;
-- Expected: user_id = NULL, session_id = 'anon-session-123'
```

---

**Test 11: Selected Text (Authenticated)**
```bash
TOKEN="<token-from-signin>"

curl -X POST http://localhost:8000/ask_selected \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer $TOKEN" \
  -d '{
    "question": "Explain this",
    "context": "ROS 2 is a middleware framework for robot communication using DDS.",
    "session_id": null
  }'
```

**Expected Response**: Success, conversation_id returned

**Database Verification**:
```sql
SELECT user_id, question_type FROM conversations WHERE conversation_id = <id>;
-- Expected: user_id = 1, question_type = 'selected_text'
```

---

### Phase 3: Security Validation

**Test 12: Token Expiration (Manual)**
1. Sign up/signin to get token
2. Wait 24 hours + 1 minute (or manually edit token expiration in database)
3. Call GET /auth/me with expired token
4. **Expected**: HTTP 401 - "Invalid or expired token"

---

**Test 13: SQL Injection Protection**
```bash
curl -X POST http://localhost:8000/auth/signin \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com'\'' OR '\''1'\''='\''1",
    "password": "anything"
  }'
```

**Expected Response**: HTTP 401 - "Invalid email or password" (NOT database error)

---

**Test 14: Password Hash Verification**
```sql
SELECT email, password_hash FROM users WHERE email = 'test@example.com';
-- Expected: password_hash starts with $2b$12$ (bcrypt with cost factor 12)
-- Example: $2b$12$abcdefghijklmnopqrstuvwxyz0123456789...
```

**Verify**:
- Hash length: 60 characters
- Prefix: `$2b$12$` (bcrypt algorithm + cost factor)
- Never matches plain password

---

**Test 15: JSONB Data Integrity**
```sql
SELECT software_background::jsonb ->> 'programming_languages' AS languages
FROM users
WHERE email = 'test@example.com';
-- Expected: Returns array as text: '["Python","C++"]'

SELECT hardware_background::jsonb ->> 'rtx_gpu_access' AS gpu_access
FROM users
WHERE email = 'test@example.com';
-- Expected: Returns 'true' (boolean stored as JSONB)
```

---

## Acceptance Checklist

Use this checklist to confirm all tests passed:

### Backend Authentication
- [ ] **Signup (Test 1)**: User created with hashed password, JWT returned
- [ ] **Password Validation (Test 2)**: Short passwords rejected
- [ ] **Email Uniqueness (Test 3)**: Duplicate emails rejected
- [ ] **Signin Valid (Test 4)**: Correct credentials return JWT, last_login updated
- [ ] **Signin Invalid (Test 5)**: Wrong password returns 401
- [ ] **Get Profile (Test 6)**: Authenticated request returns user data
- [ ] **Unauthenticated Access (Test 7-8)**: Missing/invalid tokens return 401

### Authenticated Chatbot
- [ ] **Authenticated RAG (Test 9)**: Conversations save user_id (not NULL)
- [ ] **Anonymous RAG (Test 10)**: Conversations save session_id, user_id NULL
- [ ] **Authenticated Selected Text (Test 11)**: user_id tracked for selected-text mode

### Security
- [ ] **Token Expiration (Test 12)**: Expired tokens rejected
- [ ] **SQL Injection (Test 13)**: Injection attempts handled safely
- [ ] **Password Hashing (Test 14)**: Bcrypt hashes stored, not plain text
- [ ] **JSONB Integrity (Test 15)**: Background data stored as JSONB, queryable

---

## Troubleshooting Guide

### Issue: Token verification fails immediately after signup
- **Check**: BETTER_AUTH_SECRET in `.env` matches between signup and verification
- **Fix**: Ensure `.env` loaded correctly (`python-dotenv` installed)
- **Fix**: Restart backend server after changing SECRET_KEY

### Issue: "Email already registered" on first signup
- **Check**: Database already has test data
- **Fix**: Delete test user: `DELETE FROM users WHERE email = 'test@example.com';`

### Issue: Conversations not saving user_id (always NULL)
- **Check**: Authorization header included in POST /ask request
- **Check**: Token format is "Bearer <token>" (not just token)
- **Fix**: Verify frontend Chatbot.tsx includes `Authorization` header

### Issue: Password hash stored as plain text
- **Check**: auth_service.hash_password() called before db_service.create_user()
- **Fix**: Verify bcrypt installed: `pip install passlib[bcrypt]`

### Issue: JSONB columns stored as text
- **Check**: Database migration created columns as JSONB (not TEXT)
- **Fix**: Recreate tables with correct types:
```sql
ALTER TABLE users
ALTER COLUMN software_background TYPE JSONB USING software_background::jsonb,
ALTER COLUMN hardware_background TYPE JSONB USING hardware_background::jsonb;
```

---

## Success Metrics

- **100% Test Pass Rate**: All 15 tests pass on first run
- **Zero Plain Passwords**: All password_hash values are bcrypt hashes
- **Authenticated Tracking**: 100% of logged-in user conversations have user_id populated
- **Token Security**: Expired/invalid tokens always return 401 (never 500 errors)

---

## Post-Testing Actions

1. **Security Audit**: Review auth_service.py and auth.py for security vulnerabilities
2. **Performance Benchmark**: Measure P95 latency for /auth/signup and /auth/signin
3. **Documentation**: Document token expiration policy and password requirements
4. **User Testing**: Share signup/signin flow with beta testers for UX feedback
