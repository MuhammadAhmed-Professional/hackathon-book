---
id: phr-20251127-002-plan
title: Physical AI Textbook Implementation Plan
stage: plan
date: 2025-11-27
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-physical-ai-book
branch: 002-physical-ai-book
user: talal
command: /sp.plan
labels: [plan, hackathon, physical-ai, neon-postgres, better-auth, rag]
links:
  spec: specs/002-physical-ai-book/spec.md
  plan: specs/002-physical-ai-book/plan.md
  tasks: null
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-physical-ai-book/plan.md
  - specs/002-physical-ai-book/research.md
  - specs/002-physical-ai-book/data-model.md
  - specs/002-physical-ai-book/quickstart.md
  - specs/002-physical-ai-book/contracts/openapi.yaml
tests:
  - N/A (planning phase)
---

## Prompt

User command: `/sp.plan` (no arguments, operates on spec from specs/002-physical-ai-book/spec.md)

**Context**: After creating comprehensive specification for Physical AI textbook hackathon project, proceed to create implementation plan with architecture decisions, research findings, database schema, API contracts, and setup documentation.

**User Confirmation**: "no please continue" (when Task tool was interrupted)

## Response snapshot

Created comprehensive implementation plan and supporting artifacts for Physical AI & Humanoid Robotics textbook project (002-physical-ai-book):

**Planning Artifacts Created (5 files)**:

1. **plan.md** (Implementation Plan):
   - **Summary**: Textbook with RAG + Neon Postgres + Better-auth, reusing 60% of 001-hackathon-app infrastructure
   - **Technical Context**: Python 3.11+, Node.js 18+, FastAPI, Docusaurus, Qdrant, Neon Postgres, Better-auth, OpenAI
   - **Constitution Check**: ✅ All pre-research gates pass (spec-driven, reusable intelligence, security, architecture documentation)
   - **Project Structure**: Documented reuse strategy (REUSE vs NEW vs EXTEND for each component)
   - **Complexity Tracking**: No violations, architecture follows constitution
   - **ADR Requirements**: 4 new ADRs identified (Neon Postgres, Better-auth, textbook structure, JSONB columns) + 4 reused from 001-hackathon-app
   - **Implementation Phases**: 5 phases with time estimates (48-60 hours total)
   - **Risk Analysis**: 8 risks identified with mitigation strategies (content creation time, Neon integration, Better-auth complexity, deadline pressure)
   - **Success Metrics**: Hackathon submission criteria + quality metrics + 200-point breakdown

2. **research.md** (Phase 0 Research Findings):
   - **Physical AI Content**: 4 modules with key concepts, learning objectives (13 weeks), chapter structure (800-1,200 words each)
   - **Neon Postgres Integration**: Connection patterns (psycopg2-binary), schema design (users, conversations tables), JSONB usage, connection pooling
   - **Better-auth Integration**: Setup with React/Docusaurus, JWT management (python-jose, passlib), authentication flows (signup, signin, protected endpoints)
   - **Textbook Structure**: Nested Docusaurus directories (module1/, module2/, module3/, module4/), sidebar navigation, internal linking
   - **4 Research Decisions**: Content depth (800-1,200 words), JSONB vs normalized tables, localStorage vs httpOnly cookies, flat vs nested structure

3. **data-model.md** (Database Schema):
   - **Users Table**: id, email (unique), password_hash (bcrypt), software_background (JSONB), hardware_background (JSONB), created_at, last_login
   - **Conversations Table**: id, user_id (FK nullable), session_id, question, answer, citations (JSONB), question_type (enum), timestamp
   - **Entity-Relationship Diagram**: 1:N relationship (users → conversations), ON DELETE SET NULL for user deletion
   - **JSONB Schemas**: software_background (programming_languages[], robotics_experience, ai_ml_level), hardware_background (rtx_gpu_access, rtx_gpu_model, jetson_kit, robot_hardware), citations (array of {module, chapter, chunk_id, relevance_score})
   - **Validation Rules**: Email regex, password minimum 8 chars, question_type enum, user_id OR session_id required
   - **Migration Scripts**: 001_initial.sql for creating tables with indexes

4. **quickstart.md** (Setup Guide):
   - **10 Parts**: Clone repo → Set up Neon → Set up Qdrant → Configure .env → Backend setup → Frontend setup → Ingest content → Test RAG → Test auth → Build for production
   - **Prerequisites**: Python 3.11+, Node 18+, Neon account, Qdrant account, OpenAI API key
   - **Troubleshooting**: 8 common issues with solutions (Qdrant connection, OpenAI rate limits, Neon SSL, CORS errors, memory errors, JWT errors)
   - **Development Workflow**: Daily routine for starting backend/frontend, making changes, testing, committing
   - **Deployment**: GitHub Pages for frontend, Render/Railway for backend

5. **contracts/openapi.yaml** (API Specification):
   - **11 Endpoints**: /ping (health), /ingest (content), /ask (RAG), /ask_selected (selected-text), /auth/signup, /auth/signin, /auth/me, /profile, /skills/summarize, /skills/quiz, /skills/explain
   - **Security**: JWT Bearer authentication for protected endpoints
   - **Schemas**: 13 components (Error, AskResponse, AskSelectedResponse, Citation, SignupRequest, SoftwareBackground, HardwareBackground, AuthResponse, UserProfile, etc.)
   - **Request/Response Examples**: Full examples for all endpoints with validation rules

**Key Architecture Decisions**:

1. **Infrastructure Reuse (60%)**:
   - REUSE: RAG pipeline, chatbot UI, agent skills (3 skills with P+Q+P), Qdrant integration, FastAPI structure
   - EXTEND: /ask and /ask_selected endpoints (add Neon Postgres saves), config.py (add DB connection)
   - NEW: Neon Postgres layer (db_service.py, connection pool), Better-auth (auth_service.py, JWT), Physical AI textbook content (20+ chapters)

2. **Neon Serverless Postgres**:
   - **Rationale**: Serverless (no infrastructure), free tier (3GB, 100 compute hours), PostgreSQL compatibility, JSONB support, excellent FastAPI integration
   - **Schema**: JSONB columns for flexible user backgrounds and conversation citations (agile, easy to extend)
   - **Connection**: psycopg2-binary with connection pooling (SimpleConnectionPool, 10 max connections)

3. **Better-auth**:
   - **Rationale**: Hackathon requirement, modern TypeScript-first library, easy React integration, JWT built-in, lightweight
   - **Implementation**: Simplified approach - Better-auth for UI components only, custom JWT backend with python-jose and passlib
   - **Token Storage**: localStorage for development (easier debugging), httpOnly cookies for production (security)

4. **Textbook Content Structure**:
   - **Organization**: 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA) + intro + hardware, nested directories (module1/, module2/, etc.)
   - **Chapters**: 20+ chapters, 800-1,200 words each, code examples with syntax highlighting, diagrams (architecture, flowcharts, hardware)
   - **Sidebar**: Collapsible categories for modules with Docusaurus sidebar.js configuration

**Implementation Timeline** (48-60 hours for solo developer):
- Phase 1 (Neon + Better-auth): 8-10 hours
- Phase 2 (Physical AI content): 16-20 hours (highest effort, use AI assistance)
- Phase 3 (Frontend auth UI): 6-8 hours
- Phase 4 (Integration & testing): 6-8 hours
- Phase 5 (Deployment & demo): 4-6 hours

**Risk Mitigation**:
- **Content creation time**: Prioritize Module 1 (ROS 2) and Module 3 (Isaac), use Claude Code for drafts, accept shorter chapters if needed
- **Neon integration**: Test connection early, use retry logic, fall back to SQLite for local testing
- **Better-auth complexity**: Follow docs closely, test JWT with Postman before UI, simplify to basic JWT if needed
- **Deadline pressure**: Strict prioritization (base 100 points first), timebox each phase, skip Better-auth if behind schedule

**Point Target**: 200 points
- Base (100): Textbook + RAG + Selected-text + Neon Postgres
- Better-auth (50): Signup/signin + user background questionnaire
- Agent skills (50): Already implemented from 001-hackathon-app (SummarizeSection, GenerateQuizQuestions, ExplainTerm with P+Q+P)

## Outcome

- ✅ Impact: Comprehensive implementation plan with architecture decisions, database schema, API contracts, and setup guide. Reuses 60% of existing infrastructure for efficiency. Clear phase breakdown with time estimates.
- 🧪 Tests: N/A (planning phase)
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md, contracts/openapi.yaml (5 artifacts)
- 🔁 Next prompts: Run `/sp.tasks` to generate dependency-ordered task breakdown, then begin implementation starting with Neon Postgres and Better-auth (Phase 1)
- 🧠 Reflection: Plan maximizes efficiency by reusing proven components from 001-hackathon-app. Focus on new high-value work: Physical AI content creation (20+ chapters), Neon Postgres integration, Better-auth with user profiling. Research decisions resolved key unknowns (JSONB vs normalized, localStorage vs cookies, nested vs flat structure). Risk analysis identifies content creation as highest time investment - mitigate with AI assistance and prioritization. Timeline realistic for 72-hour hackathon with full-day availability.

## Evaluation notes (flywheel)

- Failure modes observed: None (planning completed successfully)
- Graders run and results (PASS/FAIL): Plan quality validation PASS (all sections complete)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to `/sp.tasks` to break down implementation into small, testable tasks with dependency ordering. Then begin Phase 1 (Neon Postgres + Better-auth foundation) before Phase 2 (content creation).
