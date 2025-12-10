# Tasks: Hackathon App - Docusaurus Book with RAG Chatbot

**Input**: Design documents from `/specs/001-hackathon-app/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: Tests are OPTIONAL - not explicitly requested in feature specification, so test tasks are not included.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/` (as per plan.md structure)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create project structure: backend/ and frontend/ directories at repository root
- [x] T002 [P] Initialize backend Python project: Create backend/requirements.txt with FastAPI, uvicorn, qdrant-client, openai, python-dotenv, tiktoken, pytest, httpx
- [x] T003 [P] Initialize frontend Docusaurus project: Run `npx create-docusaurus@latest frontend classic` in repository root
- [ ] T004 [P] Create backend/.env.example with OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY placeholders
- [x] T005 [P] Create backend/.gitignore to exclude venv/, .env, __pycache__/, *.pyc
- [x] T006 [P] Create frontend/.gitignore for Docusaurus defaults (node_modules/, .docusaurus/, build/)
- [x] T007 [P] Create root .gitignore combining backend and frontend ignores
- [x] T008 Create root README.md with project overview and links to quickstart.md

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T009 Create backend/src/ directory structure: models/, services/, api/, config.py
- [x] T010 [P] Create backend/src/config.py for environment variable loading (OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY)
- [x] T011 [P] Create backend/src/api/main.py with FastAPI app instance and CORS middleware configuration
- [x] T012 [P] Create backend/src/api/__init__.py
- [x] T013 [P] Create backend/src/models/__init__.py
- [x] T014 [P] Create backend/src/services/__init__.py
- [ ] T015 Implement error handling infrastructure in backend/src/api/main.py (exception handlers for Qdrant and OpenAI errors)
- [x] T016 Setup logging configuration in backend/src/config.py (structured logging for debugging)
- [x] T017 Create frontend/src/components/ directory for React components
- [x] T018 [P] Configure Docusaurus for GitHub Pages deployment in frontend/docusaurus.config.js (url, baseUrl, organizationName, projectName)
- [x] T019 [P] Create frontend/src/config.js for API_BASE_URL configuration (defaults to http://localhost:8000)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Access and Navigate Educational Book (Priority: P1) 🎯 MVP

**Goal**: Deploy a Docusaurus book site with 7 chapters about AI-Driven and Spec-Driven Development, accessible via GitHub Pages with clear navigation

**Independent Test**: Deploy book to GitHub Pages, access via URL, navigate between all chapters, verify all internal links work. Book delivers educational value independently of chatbot.

### Implementation for User Story 1

- [x] T020 [US1] Create frontend/docs/intro.md with Introduction chapter content about AI-Driven and Spec-Driven Development
- [x] T021 [US1] Create frontend/docs/ai-driven-development.md with AI-Driven Development chapter content
- [x] T022 [US1] Create frontend/docs/spec-driven-development.md with Spec-Driven Development chapter content
- [x] T023 [US1] Create frontend/docs/rag-fundamentals.md with RAG Fundamentals chapter content
- [x] T024 [US1] Create frontend/docs/implementation-guide.md with Implementation Guide chapter content
- [x] T025 [US1] Create frontend/docs/chatbot-usage.md with How to Use the Chatbot chapter content
- [x] T026 [US1] Create frontend/docs/future-work.md with Future Work chapter content
- [x] T027 [US1] Configure sidebar navigation in frontend/sidebars.js to include all 7 chapters in order
- [ ] T028 [US1] Add internal links between chapters in markdown content (cross-references using Docusaurus link syntax)
- [ ] T029 [US1] Test local build: Run `npm run build` in frontend/ directory and verify no errors
- [ ] T030 [US1] Test local server: Run `npm start` in frontend/ directory and verify all chapters accessible
- [ ] T031 [US1] Configure GitHub Pages deployment: Update frontend/docusaurus.config.js with GitHub Pages settings
- [ ] T032 [US1] Deploy to GitHub Pages: Push to repository and configure GitHub Pages source (or use GitHub Actions workflow)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently - book is deployed and navigable

---

## Phase 4: User Story 2 - Ask Questions About the Book Using RAG (Priority: P2)

**Goal**: Implement RAG backend and chatbot UI that answers questions about the book using vector search and OpenAI ChatKit SDK

**Independent Test**: Ingest book content, ask questions about book content, verify answers are based on book context with source citations. Delivers value even without selected-text mode.

### Implementation for User Story 2

- [x] T033 [US2] Create backend/src/models/chunk.py with TextChunk model (chunk_id, text, embedding, chapter_id, chunk_index, token_count, metadata)
- [x] T034 [US2] Create backend/src/models/question.py with Question model (question_id, question_text, mode, timestamp, session_id)
- [x] T035 [US2] Create backend/src/models/answer.py with Answer model (answer_id, answer_text, sources, confidence, question_id, timestamp, model_used, error_message)
- [x] T036 [US2] Create backend/src/models/source_citation.py with SourceCitation model (chunk_id, chapter_id, relevance_score, snippet)
- [x] T037 [US2] Create backend/src/services/ingestion_service.py with chunk_text() function (semantic chunking: 200-300 tokens, 50 token overlap using tiktoken)
- [x] T038 [US2] Create backend/src/services/ingestion_service.py with create_embeddings() function (calls OpenAI text-embedding-3-small API)
- [x] T039 [US2] Create backend/src/services/ingestion_service.py with store_chunks() function (stores chunks in Qdrant Cloud collection "book_chunks")
- [x] T040 [US2] Create backend/src/services/ingestion_service.py with ingest_book() function (orchestrates: read markdown files, chunk, embed, store)
- [x] T041 [US2] Create backend/src/services/rag_service.py with query_qdrant() function (vector similarity search, top 5 chunks, cosine similarity)
- [x] T042 [US2] Create backend/src/services/rag_service.py with generate_answer() function (uses OpenAI ChatKit SDK with retrieved chunks as context)
- [x] T043 [US2] Create backend/src/services/rag_service.py with ask_question() function (orchestrates: embed question, query Qdrant, generate answer, return with sources)
- [x] T044 [US2] Create backend/src/api/endpoints/ingest.py with POST /ingest endpoint (calls ingestion_service.ingest_book())
- [x] T045 [US2] Create backend/src/api/endpoints/ask.py with POST /ask endpoint (calls rag_service.ask_question(), handles errors gracefully)
- [x] T046 [US2] Create backend/src/api/endpoints/ping.py with GET /ping endpoint (health check)
- [x] T047 [US2] Register all endpoints in backend/src/api/main.py (include_router for ingest, ask, ping)
- [x] T048 [US2] Create frontend/src/components/Chatbot.tsx with chat interface (input field, send button, message display area)
- [x] T049 [US2] Create frontend/src/components/Chatbot.tsx with API integration (calls /ask endpoint, displays answers and source citations)
- [x] T050 [US2] Embed Chatbot component in frontend/docs/chatbot-usage.md using MDX syntax
- [x] T051 [US2] Add CSS styling for Chatbot component in frontend/src/css/custom.css (responsive, user-friendly design)
- [ ] T052 [US2] Test ingestion: Run POST /ingest endpoint and verify chunks stored in Qdrant
- [ ] T053 [US2] Test RAG query: Ask question via /ask endpoint and verify answer with sources
- [ ] T054 [US2] Test error handling: Simulate Qdrant/OpenAI errors and verify user-friendly error messages

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - book is deployed and RAG chatbot is functional

---

## Phase 5: User Story 3 - Ask Questions About Selected Text (Priority: P3)

**Goal**: Implement selected-text question mode where users can highlight text and ask questions using only that selected context

**Independent Test**: Select text in book, click "Ask about this selection", ask question, verify answer uses only selected text (no RAG query). Delivers value by enabling focused queries.

### Implementation for User Story 3

- [x] T055 [US3] Create backend/src/api/endpoints/ask_selected.py with POST /ask_selected endpoint (accepts question and context, uses OpenAI ChatKit SDK with only provided context)
- [x] T056 [US3] Register /ask_selected endpoint in backend/src/api/main.py
- [x] T057 [US3] Create frontend/src/components/SelectedTextHandler.tsx with text selection detection (window.getSelection() API)
- [x] T058 [US3] Create frontend/src/components/SelectedTextHandler.tsx with "Ask about this selection" button (appears when text is selected)
- [x] T059 [US3] Create frontend/src/components/SelectedTextHandler.tsx with context passing (passes selected text to Chatbot component)
- [x] T060 [US3] Update frontend/src/components/Chatbot.tsx to accept selectedText prop and call /ask_selected endpoint when selectedText is provided
- [ ] T061 [US3] Embed SelectedTextHandler component in all frontend/docs/*.md files using MDX
- [x] T062 [US3] Add CSS styling for selected-text UI in frontend/src/css/custom.css (highlight style, button positioning)
- [ ] T063 [US3] Test selected-text mode: Select text, ask question, verify answer uses only selected context (no Qdrant query performed)
- [ ] T064 [US3] Test validation: Verify empty or too-short selected text is rejected with clear message
- [ ] T065 [US3] Test error handling: Verify clear message when answer cannot be found in selected context

**Checkpoint**: At this point, User Stories 1, 2, AND 3 should all work independently - book, RAG chatbot, and selected-text mode are functional

---

## Phase 6: User Story 4 - Use Reusable Agent Skills (Priority: P4 - Bonus)

**Goal**: Implement three reusable agent skills (SummarizeSection, GenerateQuizQuestions, ExplainTerm) using Claude Code Subagents pattern with Persona + Questions + Principles

**Independent Test**: Invoke each skill with book text, verify outputs match skill purposes, confirm skills are documented with persona, questions, and principles. Delivers value through specialized capabilities.

### Implementation for User Story 4

- [x] T066 [US4] Create backend/src/services/skills/summarize_section.py with SummarizeSection skill (persona: educational content specialist, questions: what are key points, what is main message, principles: concise, accurate, educational)
- [x] T067 [US4] Create backend/src/services/skills/generate_quiz.py with GenerateQuizQuestions skill (persona: educational assessment designer, questions: what concepts to test, what difficulty level, principles: clear questions, plausible distractors, one correct answer)
- [x] T068 [US4] Create backend/src/services/skills/explain_term.py with ExplainTerm skill (persona: educational content explainer, questions: what is term, how is it used in context, principles: simple language, book context, examples)
- [x] T069 [US4] Document SummarizeSection skill in backend/src/services/skills/summarize_section.py with persona, questions, and principles as docstrings
- [x] T070 [US4] Document GenerateQuizQuestions skill in backend/src/services/skills/generate_quiz.py with persona, questions, and principles as docstrings
- [x] T071 [US4] Document ExplainTerm skill in backend/src/services/skills/explain_term.py with persona, questions, and principles as docstrings
- [x] T072 [US4] Create backend/src/api/endpoints/skills.py with POST /skills/summarize endpoint (calls SummarizeSection skill)
- [x] T073 [US4] Create backend/src/api/endpoints/skills.py with POST /skills/quiz endpoint (calls GenerateQuizQuestions skill)
- [x] T074 [US4] Create backend/src/api/endpoints/skills.py with POST /skills/explain endpoint (calls ExplainTerm skill, uses RAG to find term in book context)
- [x] T075 [US4] Register skills endpoints in backend/src/api/main.py
- [ ] T076 [US4] Test SummarizeSection: Invoke with book text, verify concise summary returned
- [ ] T077 [US4] Test GenerateQuizQuestions: Invoke with book text, verify multiple-choice questions returned
- [ ] T078 [US4] Test ExplainTerm: Invoke with term, verify explanation using book context returned
- [x] T079 [US4] Create ADR document in history/adr/agent-skills-design.md documenting Persona + Questions + Principles pattern usage

**Checkpoint**: All user stories should now be independently functional - book, RAG chatbot, selected-text mode, and agent skills are complete

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T080 [P] Update root README.md with complete project documentation and setup instructions
- [ ] T081 [P] Add inline code comments and docstrings to all backend Python files (educational clarity)
- [ ] T082 [P] Add inline code comments to all frontend React components (educational clarity)
- [ ] T083 Code cleanup: Remove any unused imports, variables, or functions
- [ ] T084 Performance optimization: Verify RAG query response times meet <5s goal (90% of requests)
- [x] T085 Security hardening: Verify all secrets are environment-based, no hard-coded credentials
- [ ] T086 Run quickstart.md validation: Follow quickstart guide step-by-step and verify all steps work
- [x] T087 Create ADR document in history/adr/docusaurus-choice.md documenting why Docusaurus was chosen
- [x] T088 Create ADR document in history/adr/fastapi-choice.md documenting why FastAPI was chosen
- [x] T089 Create ADR document in history/adr/rag-strategy.md documenting RAG implementation strategy and chunking approach
- [x] T090 Create ADR document in history/adr/deployment-approach.md documenting GitHub Pages deployment and backend hosting strategy

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3 → P4)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Requires User Story 1 for book content to ingest, but can be developed in parallel
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Requires User Story 2's Chatbot component, but selected-text logic is independent
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - Independent, but ExplainTerm skill benefits from RAG infrastructure (US2)

### Within Each User Story

- Models before services
- Services before endpoints
- Backend endpoints before frontend components
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T002-T007)
- All Foundational tasks marked [P] can run in parallel (T010-T014, T018-T019)
- Once Foundational phase completes, User Stories 1 and 2 can start in parallel (different components)
- Models within User Story 2 marked [P] can run in parallel (T033-T036)
- Services within User Story 2 marked [P] can run in parallel (T037-T043)
- Endpoints within User Story 2 marked [P] can run in parallel (T044-T046)
- Polish tasks marked [P] can run in parallel (T080-T082, T087-T090)

---

## Parallel Example: User Story 2

```bash
# Launch all models for User Story 2 together:
Task: "Create backend/src/models/chunk.py with TextChunk model"
Task: "Create backend/src/models/question.py with Question model"
Task: "Create backend/src/models/answer.py with Answer model"
Task: "Create backend/src/models/source_citation.py with SourceCitation model"

# Launch all endpoints for User Story 2 together:
Task: "Create backend/src/api/endpoints/ingest.py with POST /ingest endpoint"
Task: "Create backend/src/api/endpoints/ask.py with POST /ask endpoint"
Task: "Create backend/src/api/endpoints/ping.py with GET /ping endpoint"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Docusaurus book)
4. **STOP and VALIDATE**: Deploy book to GitHub Pages, test navigation
5. Demo: Show deployed book with all chapters accessible

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Deploy book → Demo (MVP!)
3. Add User Story 2 → Ingest content, test RAG → Demo
4. Add User Story 3 → Test selected-text mode → Demo
5. Add User Story 4 → Test agent skills → Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Docusaurus book)
   - Developer B: User Story 2 (RAG backend - can start after US1 content ready)
   - Developer C: User Story 3 (Selected-text mode - can start after US2 Chatbot ready)
   - Developer D: User Story 4 (Agent skills - independent)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- All file paths are relative to repository root
- Backend tasks assume Python virtual environment is activated
- Frontend tasks assume npm dependencies are installed

