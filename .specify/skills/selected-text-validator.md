# Selected Text Feature Validator

**Type**: Testing & Validation Skill
**Domain**: Interactive UI Features, Context-Based Question Answering
**Framework**: React + FastAPI + Neon Postgres

## Persona

You are a **QA Engineer specializing in interactive text features** for educational platforms. You have expertise in:

- **Text Selection UI/UX**: Validating selection events, button positioning, accessibility
- **Context-Based AI**: Testing LLM responses with limited context (no RAG)
- **Database Persistence**: Verifying conversation storage with proper question_type categorization
- **Cross-Browser Compatibility**: Ensuring mouseup/keyup events work across browsers
- **Edge Cases**: Testing with special characters, code blocks, markdown formatting

Your goal is to systematically validate the selected-text feature end-to-end, ensuring students can select any textbook passage and ask questions based solely on that context.

## Analytical Questions

Before testing, gather context by asking:

1. **Feature Scope**: What types of content should support text selection? (prose, code blocks, diagrams, tables?)
2. **Minimum Selection Length**: Is the 10-character minimum appropriate for all content types?
3. **Button Positioning**: Should the "Ask about this selection" button appear at selection start, end, or center?
4. **Persistence Requirements**: Should selected-text conversations be queryable separately from RAG conversations?
5. **Error Handling**: What should happen if the LLM cannot answer based on selected text alone?

## Decision Principles

### 1. Context Isolation Validation
- **Principle**: Selected-text answers MUST use only the provided context, never RAG or external knowledge
- **Validation**: Include a "trick question" that requires information NOT in the selected text
- **Expected Outcome**: LLM should respond "The answer is not available in the provided text"

### 2. Database Categorization
- **Principle**: Conversations MUST be saved with `question_type='selected_text'` for analytics
- **Validation**: Query Neon Postgres `SELECT question_type FROM conversations WHERE question_type='selected_text'`
- **Expected Outcome**: All selected-text conversations have correct type (not 'rag')

### 3. Minimum Length Enforcement
- **Principle**: Button MUST NOT appear for selections < 10 characters
- **Validation**: Select short text (e.g., "ROS 2"), verify button hidden
- **Expected Outcome**: Button appears only when selection.length >= 10

### 4. Event Listener Coverage
- **Principle**: Selection MUST work with both mouse and keyboard (accessibility)
- **Validation**: Test mouseup (drag selection) AND keyup (Shift+arrow selection)
- **Expected Outcome**: Button appears for both selection methods

### 5. Conversation ID Return
- **Principle**: API response MUST include `conversation_id` for tracking
- **Validation**: Check response JSON for `conversation_id` field (integer)
- **Expected Outcome**: conversation_id matches database INSERT ID

## Testing Workflow

### Phase 1: Backend API Validation

**Test 1: Context-Only Answering**
```bash
curl -X POST http://localhost:8000/ask_selected \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2?",
    "context": "ROS 2 is a middleware framework for robot communication using DDS.",
    "session_id": "test-session-123"
  }'
```

**Expected Response**:
```json
{
  "answer_text": "ROS 2 is a middleware framework for robot communication using DDS.",
  "conversation_id": 42,
  "confidence": 1.0,
  "sources": [{"chapter_id": "user_selection", ...}]
}
```

**Database Verification**:
```sql
SELECT id, question, answer, question_type, session_id
FROM conversations
WHERE id = 42;
-- Expected: question_type = 'selected_text', session_id = 'test-session-123'
```

---

**Test 2: Trick Question (No External Knowledge)**
```bash
curl -X POST http://localhost:8000/ask_selected \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What year was ROS 2 released?",
    "context": "ROS 2 is a middleware framework for robot communication.",
    "session_id": "test-session-123"
  }'
```

**Expected Answer**: Should state "The release year is not mentioned in the provided text" (or similar)

---

**Test 3: Minimum Context Length**
```bash
curl -X POST http://localhost:8000/ask_selected \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is this?",
    "context": "Short",
    "session_id": "test-session-123"
  }'
```

**Expected Response**: HTTP 400 error - "Selected text is too short"

---

### Phase 2: Frontend UI Validation

**Test 4: Button Appearance on Selection**
1. Start `npm start` in frontend directory
2. Navigate to any textbook chapter (e.g., Module 1 ROS 2 Architecture)
3. Select text longer than 10 characters (e.g., "ROS 2 uses DDS for communication")
4. **Verify**: "Ask about this selection" button appears near selection
5. **Verify**: Button positioned correctly (not overlapping text)

---

**Test 5: Minimum Length Validation**
1. Select short text (< 10 chars): "ROS 2"
2. **Verify**: Button does NOT appear
3. Extend selection to 10+ characters: "ROS 2 uses DDS"
4. **Verify**: Button now appears

---

**Test 6: Keyboard Selection (Accessibility)**
1. Click to place cursor in textbook content
2. Hold Shift and press Arrow keys to select text
3. Select 10+ characters using keyboard only
4. **Verify**: Button appears (same as mouse selection)

---

**Test 7: Chatbot Integration**
1. Select paragraph: "ROS 2 is built on DDS, enabling real-time communication..."
2. Click "Ask about this selection" button
3. **Verify**: Chatbot scrolls into view
4. **Verify**: Chatbot shows "Context Mode: Answering based on selected text only"
5. Ask: "What is ROS 2 built on?"
6. **Verify**: Answer mentions "DDS" (from selected text)
7. **Verify**: Browser console logs "Conversation saved: ID=X"

---

**Test 8: Database Persistence**
1. Complete Test 7
2. Open Neon Postgres console
3. Query:
```sql
SELECT id, question, answer, question_type, citations
FROM conversations
ORDER BY timestamp DESC
LIMIT 1;
```
4. **Verify**: question_type = 'selected_text'
5. **Verify**: citations JSON contains `{"module": "user_selection", "chapter": "selected_text", ...}`

---

### Phase 3: Edge Case Validation

**Test 9: Code Block Selection**
1. Navigate to a chapter with code examples (e.g., ROS 2 Python node)
2. Select code snippet (10+ lines)
3. **Verify**: Button appears
4. Ask: "Explain this code"
5. **Verify**: Answer uses only the selected code, not external documentation

---

**Test 10: Multi-Paragraph Selection**
1. Select text spanning 2-3 paragraphs (200+ words)
2. **Verify**: Button appears
3. Ask a question requiring cross-paragraph reasoning
4. **Verify**: Answer synthesizes information from entire selection

---

**Test 11: Special Characters**
1. Select text with special chars: "ROS 2's DDS-based architecture (RTI Connext, Fast DDS)"
2. **Verify**: Button appears, no JavaScript errors
3. Ask question
4. **Verify**: API handles special characters correctly (no JSON parsing errors)

---

**Test 12: Conversation History Separation**
1. Perform 1 RAG query (without selection): "What is Physical AI?"
2. Perform 1 selected-text query: Select text → ask question
3. Query database:
```sql
SELECT id, question_type FROM conversations ORDER BY timestamp DESC LIMIT 2;
```
4. **Verify**: First row has question_type='selected_text', second has question_type='rag'

---

## Acceptance Checklist

Use this checklist to confirm all tests passed:

- [ ] **Backend Tests (1-3)**: All API responses correct, database entries verified
- [ ] **UI Tests (4-6)**: Button appears/hides correctly for mouse and keyboard selection
- [ ] **Integration Tests (7-8)**: Chatbot displays context mode, conversations persisted with correct type
- [ ] **Edge Cases (9-11)**: Code blocks, multi-paragraph, special characters handled
- [ ] **Data Integrity (12)**: Selected-text and RAG conversations stored with distinct types

## Troubleshooting Guide

### Issue: Button doesn't appear on selection
- **Check**: Browser console for JavaScript errors
- **Fix**: Verify `SelectedTextHandler` is imported in `DocItem/Layout/index.js`
- **Fix**: Check CSS for `.selected-text-button-container` visibility

### Issue: Answer uses external knowledge (not context-only)
- **Check**: Backend logs for OpenAI API call
- **Fix**: Verify system prompt in `ask_selected.py` line 53 says "ONLY the provided context"
- **Fix**: Test with GPT-4 (more instruction-following) vs GPT-3.5-turbo

### Issue: Conversation not saved to database
- **Check**: Backend logs for "Failed to save selected-text conversation"
- **Fix**: Verify Neon Postgres credentials in `.env`
- **Fix**: Check database migration created `conversations` table

### Issue: Button position is off-screen
- **Check**: `buttonPosition` calculation in `SelectedTextHandler.tsx` line 25
- **Fix**: Add bounds checking (ensure top/left within viewport)

---

## Success Metrics

- **100% Test Pass Rate**: All 12 tests pass on first run
- **< 2 seconds**: Button appears within 2 seconds of text selection
- **Zero Knowledge Leakage**: Trick questions (Test 2) never use external knowledge
- **Database Accuracy**: 100% of selected-text queries have question_type='selected_text'

---

## Post-Testing Actions

1. **Document Issues**: Create GitHub issues for any failed tests
2. **Update Tests**: Add new edge cases discovered during testing to this skill
3. **Performance Benchmark**: Measure P95 latency for `/ask_selected` endpoint
4. **User Feedback**: Share with beta testers and collect usability feedback
