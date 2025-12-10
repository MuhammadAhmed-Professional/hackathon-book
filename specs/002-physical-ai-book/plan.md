# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `002-physical-ai-book` | **Date**: 2025-11-27 | **Spec**: [spec.md](./spec.md)
**Hackathon**: Panaversity Physical AI & Humanoid Robotics Course Textbook
**Deadline**: Sunday, Nov 30, 2025 at 06:00 PM
**Target**: 300 points (Base 100 + Better-auth 50 + Personalization 50 + Urdu 50 + Agent Skills 50)

## Summary

This project implements a comprehensive educational textbook for teaching Physical AI & Humanoid Robotics with integrated RAG capabilities. The system consists of: (1) A Docusaurus-based textbook covering 13 weeks of course content (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA) deployed to GitHub Pages, (2) A FastAPI backend with RAG using Qdrant Cloud and OpenAI with conversation persistence in Neon Serverless Postgres, (3) Better-auth authentication with user background questionnaire, (4) Selected-text mode for focused queries, and (5) Reusable agent skills (SummarizeSection, GenerateQuizQuestions, ExplainTerm) following P+Q+P pattern. The implementation reuses 60% of infrastructure from feature 001-hackathon-app, focusing new development on Physical AI textbook content, Neon Postgres integration, and Better-auth with user profiling.

## Technical Context

**Language/Version**:
- Python 3.11+ (for FastAPI backend)
- Node.js 18+ (for Docusaurus frontend)
- TypeScript 5.x (for React components and Better-auth integration)

**Primary Dependencies**:
- Frontend: Docusaurus 3.x, React 19.x, Better-auth 1.x, React DOM
- Backend: FastAPI 0.109+, uvicorn, qdrant-client 1.7+, openai 1.12+ (Agents SDK/ChatKit SDK), psycopg2-binary 2.9+ (Neon Postgres driver), python-dotenv, tiktoken, bcrypt/passlib (password hashing)
- Databases: Qdrant Cloud Free Tier (vectors), Neon Serverless Postgres (conversations, users)
- AI Services: OpenAI API (text-embedding-3-small for embeddings, gpt-3.5-turbo for chat)

**Storage**:
- Qdrant Cloud Free Tier for textbook content embeddings and vector search
- Neon Serverless Postgres for user profiles (email, password_hash, software_background JSON, hardware_background JSON) and conversation history (question, answer, citations JSON, timestamp, user_id, session_id)
- Local file system for Docusaurus textbook content (markdown files for Physical AI modules)
- Environment variables for API keys, database credentials, and Better-auth secrets

**Testing**:
- Backend: pytest with httpx for FastAPI endpoints, pytest-asyncio for async tests
- Database: pytest fixtures for Neon Postgres test database
- Frontend: Manual testing for Better-auth flows, chatbot interactions, selected-text mode
- Integration: End-to-end tests for RAG pipeline with Neon Postgres persistence

**Target Platform**:
- Web browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+)
- GitHub Pages for textbook deployment (static site)
- Cloud-hosted FastAPI backend (Render, Railway, Fly.io, or Vercel serverless)
- Neon Serverless Postgres (cloud-hosted, no local DB required)

**Project Type**: Web application (frontend + backend with database)

**Performance Goals**:
- Textbook pages load in under 2 seconds (static generation)
- RAG query responses in under 7 seconds (p95 latency)
- Neon Postgres writes complete in under 2 seconds (99% success rate)
- Better-auth signup/signin in under 3 seconds
- Support 10-50 concurrent users (hackathon demonstration scale)

**Constraints**:
- Qdrant Cloud Free Tier limits (storage, API rate limits)
- Neon Serverless Postgres Free Tier limits (3GB storage, 100 compute hours/month)
- OpenAI API rate limits and costs (embeddings: $0.0001/1K tokens, chat: $0.002/1K tokens)
- GitHub Pages deployment constraints (static only, no server-side rendering)
- Hackathon deadline: 72 hours from start (Nov 27 → Nov 30)
- Solo developer with research requirement for Physical AI content
- All secrets environment-based (no hard-coded credentials)

**Scale/Scope**:
- Textbook: 20+ chapters across 4 modules + introduction + hardware section (estimated 15,000-25,000 words)
- Vector chunks: Estimated 800-1,200 chunks (200-300 tokens each with 50-token overlap)
- API endpoints: 7 endpoints (health, ingest, ask, ask_selected, skills, signup, signin)
- Database tables: 2 (users, conversations)
- React components: 3 (Chatbot, SelectedTextHandler, SignupForm/SigninForm)
- Agent skills: 3 skills reused from 001-hackathon-app (SummarizeSection, GenerateQuizQuestions, ExplainTerm)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Pre-Research Gates

✅ **Spec-Driven Development**: Plan follows Spec-Kit Plus workflow (Constitution → Specify → Plan → Tasks → Implement)

✅ **Reusable Intelligence**: Plan will generate ADRs for new decisions (Neon Postgres choice, Better-auth integration, textbook content structure). Reuses existing ADRs from 001-hackathon-app (Docusaurus, FastAPI, RAG strategy) with references.

✅ **AI-Driven Development**: Plan uses Claude Code for implementation with research assistance for Physical AI content. PHRs will document effective prompts for textbook content generation.

✅ **Educational Clarity**: Plan prioritizes clear, well-structured textbook content suitable for students learning Physical AI. Code reuses simple, readable implementations from 001-hackathon-app.

✅ **Security and Secrets**: Plan uses environment variables for all secrets (OpenAI API key, Qdrant URL/key, Neon Postgres connection string, Better-auth secret). Passwords hashed with bcrypt/passlib.

✅ **Architecture Decision Documentation**: Plan includes ADR creation for: (1) Neon Postgres as relational database, (2) Better-auth for authentication, (3) Textbook content structure and module organization.

### Post-Design Gates (Re-evaluated after Phases 1-3)

- [ ] New architecture decisions documented in ADRs (Neon Postgres, Better-auth, textbook structure) - **TODO: Create ADRs**
- [x] No hard-coded secrets in database connection strings or auth configuration - **VERIFIED: All secrets use environment variables, .env.example uses placeholders**
- [x] Neon Postgres schema supports user profiles with JSON columns for flexibility - **VERIFIED: users.software_background and users.hardware_background are JSONB**
- [x] Better-auth integration follows secure patterns (JWT tokens, httpOnly cookies) - **VERIFIED: auth_service.py uses python-jose for JWT, HS256 algorithm, 24-hour expiry**
- [x] Textbook content structure supports modular organization (4 modules, 13 weeks) - **VERIFIED: 4 module directories, weekly-breakdown.md maps 13 weeks, sidebar.js nested structure**
- [ ] All major technology choices for new components justified in ADRs - **TODO: Create ADRs (5/6 gates passing)**

## Project Structure

### Documentation (this feature)

```text
specs/002-physical-ai-book/
├── spec.md              # Feature specification (COMPLETED)
├── plan.md              # This file (/sp.plan output)
├── research.md          # Phase 0: Research findings (textbook content, Neon Postgres, Better-auth)
├── data-model.md        # Phase 1: Database schema (users, conversations tables)
├── quickstart.md        # Phase 1: Setup guide (Neon Postgres, Better-auth config)
├── contracts/           # Phase 1: API contracts
│   ├── openapi.yaml     # Extended FastAPI contract with auth endpoints
│   └── README.md        # Contract documentation
├── checklists/          # Quality validation
│   └── requirements.md  # Spec quality checklist (COMPLETED)
└── tasks.md             # Phase 2: Task breakdown (/sp.tasks - NOT created by /sp.plan)
```

### Source Code (repository root - REUSES 001-hackathon-app structure)

```text
# Existing structure from 001-hackathon-app (60% reuse)

backend/
├── src/
│   ├── models/          # REUSE: chunk.py, question.py, answer.py, source_citation.py
│   │   ├── user.py      # NEW: User model (id, email, password_hash, backgrounds JSON)
│   │   └── conversation.py  # NEW: Conversation model (id, user_id, question, answer, citations JSON)
│   ├── services/
│   │   ├── rag_service.py        # REUSE: RAG pipeline (query, generate)
│   │   ├── ingestion_service.py  # REUSE: Chunking, embedding, Qdrant storage
│   │   ├── db_service.py         # NEW: Neon Postgres operations (users, conversations)
│   │   ├── auth_service.py       # NEW: Better-auth integration (signup, signin, JWT)
│   │   └── skills/               # REUSE: All 3 skills (summarize, quiz, explain)
│   ├── api/
│   │   ├── main.py      # EXTEND: Add Better-auth middleware, CORS for auth
│   │   └── endpoints/
│   │       ├── ask.py              # EXTEND: Add user_id from JWT, save to Neon Postgres
│   │       ├── ask_selected.py     # EXTEND: Add user_id, save to Neon Postgres
│   │       ├── ingest.py           # REUSE: Content ingestion (no changes)
│   │       ├── ping.py             # REUSE: Health check
│   │       ├── skills.py           # REUSE: Agent skills endpoints
│   │       ├── auth.py             # NEW: /signup, /signin, /me endpoints
│   │       └── profile.py          # NEW: /profile (get user background)
│   ├── config.py        # EXTEND: Add Neon Postgres config, Better-auth secret
│   └── db/
│       ├── __init__.py  # NEW: Database initialization
│       ├── connection.py  # NEW: Neon Postgres connection pool
│       └── migrations/    # NEW: SQL schema files
│           └── 001_initial.sql  # Users and conversations tables
├── tests/
│   ├── unit/            # EXTEND: Add tests for auth_service, db_service
│   ├── integration/     # EXTEND: Add tests for /signup, /signin, conversation storage
│   └── contract/        # EXTEND: Add auth endpoint contract tests
├── requirements.txt     # EXTEND: Add psycopg2-binary, passlib, python-jose (JWT)
└── .env.example         # EXTEND: Add NEON_DATABASE_URL, BETTER_AUTH_SECRET

frontend/
├── docs/                # REPLACE: New Physical AI textbook content
│   ├── intro.md         # NEW: Introduction to Physical AI
│   ├── module1/         # NEW: Module 1 - ROS 2
│   │   ├── index.md
│   │   ├── ros2-architecture.md
│   │   ├── nodes-topics-services.md
│   │   ├── python-integration.md
│   │   └── urdf-for-humanoids.md
│   ├── module2/         # NEW: Module 2 - Gazebo & Unity
│   │   ├── index.md
│   │   ├── gazebo-simulation.md
│   │   ├── urdf-sdf-formats.md
│   │   ├── physics-simulation.md
│   │   └── unity-rendering.md
│   ├── module3/         # NEW: Module 3 - NVIDIA Isaac
│   │   ├── index.md
│   │   ├── isaac-sim.md
│   │   ├── isaac-ros.md
│   │   ├── vslam-navigation.md
│   │   └── nav2-planning.md
│   ├── module4/         # NEW: Module 4 - VLA
│   │   ├── index.md
│   │   ├── voice-to-action.md
│   │   ├── cognitive-planning.md
│   │   └── capstone-project.md
│   ├── hardware.md      # NEW: Hardware requirements section
│   └── weekly-breakdown.md  # NEW: 13-week schedule
├── src/
│   ├── components/
│   │   ├── Chatbot.tsx  # EXTEND: Add authentication state, display user info
│   │   ├── SelectedTextHandler.tsx  # REUSE: No changes
│   │   ├── SignupForm.tsx           # NEW: Better-auth signup with questionnaire
│   │   ├── SigninForm.tsx           # NEW: Better-auth signin
│   │   ├── AuthProvider.tsx         # NEW: Better-auth context provider
│   │   └── ProfileButton.tsx        # NEW: Display logged-in user, logout
│   ├── pages/
│   │   ├── signup.tsx   # NEW: Signup page
│   │   └── signin.tsx   # NEW: Signin page
│   ├── config.js        # EXTEND: Add auth endpoints
│   └── css/
│       └── custom.css   # EXTEND: Styles for auth forms
├── static/             # REUSE: Static assets
├── docusaurus.config.js  # EXTEND: Update title, baseUrl, sidebar for Physical AI
├── sidebars.js         # REPLACE: New sidebar structure for 4 modules
├── package.json        # EXTEND: Add better-auth npm package
└── babel.config.js     # REUSE: No changes

.env.example            # EXTEND: Add Neon Postgres and Better-auth variables
.gitignore             # REUSE: No changes
README.md              # EXTEND: Update for Physical AI textbook project
```

**Structure Decision**: Reuse existing 001-hackathon-app structure (60% infrastructure already complete) and extend with new components for Physical AI textbook content, Neon Postgres integration, and Better-auth. This maximizes efficiency for the 72-hour hackathon timeline by leveraging existing RAG pipeline, chatbot UI, and agent skills. New development focuses on textbook content creation (20+ chapters), database layer (Neon Postgres), and authentication layer (Better-auth).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations identified. The architecture follows constitution principles:
- Spec-driven development via Spec-Kit Plus workflow
- Reusable intelligence via ADRs and PHRs
- Security via environment-based secrets and password hashing
- Educational clarity via well-structured textbook content and simple code reuse

**Justification for infrastructure reuse**: Reusing 60% of 001-hackathon-app code is aligned with the constitution principle "Reusable Intelligence as First-Class Artifacts." The existing RAG pipeline, chatbot, and agent skills are proven, documented, and educational—no need to rebuild from scratch.

## Phase 0: Research

### Research Tasks

1. **Physical AI Course Content Research**
   - **Task**: Research Physical AI concepts, ROS 2 fundamentals, Gazebo/Unity simulation, NVIDIA Isaac platform, and humanoid robotics to create comprehensive textbook content
   - **Sources**: ROS 2 official documentation, NVIDIA Isaac documentation, academic papers on embodied intelligence, existing robotics course materials
   - **Output**: Content outline for 20+ chapters with key concepts, learning objectives, and structure for each module

2. **Neon Serverless Postgres Integration**
   - **Task**: Research Neon Postgres connection patterns, schema design for users and conversations, psycopg2 connection pooling, async database operations with FastAPI
   - **Sources**: Neon documentation, psycopg2 docs, FastAPI database guides
   - **Output**: Database schema design, connection pool configuration, async query patterns

3. **Better-auth Integration Patterns**
   - **Task**: Research Better-auth setup with React/Docusaurus, JWT token management, signup/signin flows, password hashing, user session management
   - **Sources**: Better-auth documentation, FastAPI JWT authentication guides, React authentication patterns
   - **Output**: Authentication architecture, JWT flow diagram, signup/signin API contracts

4. **Textbook Content Structure**
   - **Task**: Research best practices for educational textbook organization, module structure for 13-week courses, chapter length, code example formats
   - **Sources**: Docusaurus best practices, educational design principles, existing technical textbooks
   - **Output**: Textbook structure template with chapter organization, sidebar navigation, internal linking strategy

### Research Decisions to Resolve

- **Content depth**: How detailed should textbook chapters be? (Target: 800-1,200 words per chapter for comprehensive coverage without overwhelming students)
- **Neon Postgres schema**: Should user backgrounds be JSON columns or separate tables? (Decision: JSON columns for flexibility, easier to extend questionnaire without migrations)
- **Better-auth token storage**: LocalStorage vs. httpOnly cookies? (Decision: httpOnly cookies for security, with localStorage fallback for development)
- **Textbook module organization**: Flat structure vs. nested directories? (Decision: Nested directories for each module to support complex navigation and code examples)

**Output**: `research.md` with consolidated findings and decisions for all research tasks

## Phase 1: Design & Contracts

### Data Model (`data-model.md`)

**Entities from spec.md**:

1. **User** (Neon Postgres `users` table):
   - `id` (SERIAL PRIMARY KEY)
   - `email` (VARCHAR(255) UNIQUE NOT NULL)
   - `password_hash` (VARCHAR(255) NOT NULL) - bcrypt hashed
   - `software_background` (JSONB) - stores: programming_languages[], robotics_experience, ai_ml_level
   - `hardware_background` (JSONB) - stores: rtx_gpu_access, rtx_gpu_model, jetson_kit, robot_hardware
   - `created_at` (TIMESTAMP DEFAULT NOW())
   - `last_login` (TIMESTAMP)

2. **Conversation** (Neon Postgres `conversations` table):
   - `id` (SERIAL PRIMARY KEY)
   - `user_id` (INTEGER REFERENCES users(id) NULLABLE) - NULL for anonymous users
   - `session_id` (VARCHAR(255)) - for anonymous tracking
   - `question` (TEXT NOT NULL)
   - `answer` (TEXT NOT NULL)
   - `citations` (JSONB) - array of {module, chapter, chunk_id, relevance_score}
   - `question_type` (VARCHAR(50)) - 'rag' or 'selected_text'
   - `timestamp` (TIMESTAMP DEFAULT NOW())

3. **Module** (Docusaurus content organization, no database):
   - Module 1: ROS 2 Robotic Nervous System (Weeks 3-5)
   - Module 2: Gazebo & Unity Digital Twin (Weeks 6-7)
   - Module 3: NVIDIA Isaac AI-Robot Brain (Weeks 8-10)
   - Module 4: Vision-Language-Action (Week 13)

4. **Chapter** (Markdown files, no database):
   - Each module contains 4-6 chapters
   - Chapters include: title, content, code examples, diagrams, learning objectives

5. **Text Chunk** (Qdrant vectors, REUSE from 001-hackathon-app):
   - `chunk_id` (UUID)
   - `text` (str)
   - `embedding` (1536-dim vector)
   - `metadata`: {module, chapter, chunk_index, token_count}

6. **Agent Skill** (REUSE from 001-hackathon-app):
   - SummarizeSection, GenerateQuizQuestions, ExplainTerm
   - Documented with P+Q+P pattern

### API Contracts (`contracts/openapi.yaml`)

**New Endpoints**:

1. **POST /auth/signup**
   - Request: `{"email": "str", "password": "str", "software_background": {...}, "hardware_background": {...}}`
   - Response: `{"user_id": int, "email": "str", "token": "JWT"}`
   - Status: 201 Created, 400 Bad Request (email exists), 500 Server Error

2. **POST /auth/signin**
   - Request: `{"email": "str", "password": "str"}`
   - Response: `{"user_id": int, "email": "str", "token": "JWT"}`
   - Status: 200 OK, 401 Unauthorized (invalid credentials), 500 Server Error

3. **GET /auth/me** (requires JWT)
   - Headers: `Authorization: Bearer <JWT>`
   - Response: `{"user_id": int, "email": "str", "software_background": {...}, "hardware_background": {...}}`
   - Status: 200 OK, 401 Unauthorized (invalid token)

4. **GET /profile** (requires JWT)
   - Headers: `Authorization: Bearer <JWT>`
   - Response: `{"email": "str", "software_background": {...}, "hardware_background": {...}, "created_at": "ISO8601"}`
   - Status: 200 OK, 401 Unauthorized

**Extended Endpoints**:

5. **POST /ask** (EXTEND to save conversations)
   - Request: `{"question": "str", "session_id": "str (optional)"}`
   - Headers: `Authorization: Bearer <JWT> (optional)`
   - Response: `{"answer_text": "str", "sources": [...], "conversation_id": int}`
   - Side effect: Save to Neon Postgres `conversations` table with user_id or session_id

6. **POST /ask_selected** (EXTEND to save conversations)
   - Request: `{"question": "str", "context": "str", "session_id": "str (optional)"}`
   - Headers: `Authorization: Bearer <JWT> (optional)`
   - Response: `{"answer_text": "str", "sources": [...], "conversation_id": int}`
   - Side effect: Save to Neon Postgres with question_type='selected_text'

**Reused Endpoints** (no changes):
- GET /ping
- POST /ingest
- POST /skills/summarize
- POST /skills/quiz
- POST /skills/explain

### Quickstart (`quickstart.md`)

**Prerequisites**:
- Python 3.11+
- Node.js 18+
- Neon Serverless Postgres account (free tier)
- Qdrant Cloud account (free tier)
- OpenAI API key
- Git

**Setup Steps**:

1. Clone repository and checkout branch `002-physical-ai-book`
2. Set up Neon Postgres:
   - Create new project at neon.tech
   - Copy connection string (postgres://user:pass@host/db)
   - Run `backend/src/db/migrations/001_initial.sql` to create tables
3. Configure environment variables:
   - Copy `.env.example` to `.env`
   - Fill: OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL, BETTER_AUTH_SECRET
4. Set up backend:
   - `cd backend`
   - `python -m venv venv`
   - `source venv/bin/activate` (Windows: `venv\Scripts\activate`)
   - `pip install -r requirements.txt`
   - `uvicorn src.api.main:app --reload`
5. Set up frontend:
   - `cd frontend`
   - `npm install`
   - `npm start`
6. Ingest textbook content:
   - `curl -X POST http://localhost:8000/ingest`
7. Test signup:
   - Navigate to `http://localhost:3000/signup`
   - Fill form with email, password, backgrounds
8. Test RAG chatbot:
   - Navigate to any textbook chapter
   - Ask question via embedded chatbot
9. Verify Neon Postgres:
   - Check `conversations` table for saved question/answer

**Troubleshooting**:
- Neon Postgres connection errors: Verify connection string format and SSL settings
- Better-auth JWT errors: Regenerate BETTER_AUTH_SECRET (use `openssl rand -hex 32`)
- CORS errors: Ensure backend CORS allows frontend origin

## ADR Requirements

### Required ADRs for this Feature

1. **ADR: Neon Serverless Postgres for Conversation Persistence**
   - **Context**: Need relational database for users and conversations. Options: PostgreSQL, MySQL, SQLite, Neon Serverless Postgres.
   - **Decision**: Neon Serverless Postgres
   - **Rationale**: Serverless (no infrastructure management), free tier (3GB storage, 100 compute hours), PostgreSQL compatibility, JSONB support for flexible user backgrounds, excellent FastAPI integration via psycopg2, automatic SSL connections
   - **Alternatives**: (1) Local PostgreSQL (requires setup, not cloud-native), (2) Supabase (more complex, includes auth built-in but we need Better-auth), (3) SQLite (not suitable for cloud deployment)
   - **Consequences**: (+) No database management, fast queries, free tier sufficient for hackathon; (-) Vendor lock-in, compute hour limits on free tier

2. **ADR: Better-auth for Authentication**
   - **Context**: Need signup/signin with user background collection. Options: Custom JWT auth, Better-auth, NextAuth.js, Supabase Auth.
   - **Decision**: Better-auth
   - **Rationale**: Hackathon requirement, modern authentication library, TypeScript-first, easy React integration, supports email/password with custom fields (backgrounds), JWT token management built-in, lightweight (no heavy dependencies)
   - **Alternatives**: (1) Custom JWT (time-consuming, error-prone), (2) NextAuth.js (requires Next.js, overkill for Docusaurus), (3) Supabase Auth (forces Supabase DB, conflicts with Neon choice)
   - **Consequences**: (+) Fast implementation, secure by default, well-documented; (-) Newer library (less community support than NextAuth), requires learning Better-auth patterns

3. **ADR: Physical AI Textbook Content Structure**
   - **Context**: Need to organize 13 weeks of Physical AI content into navigable textbook structure. Options: Flat chapter list, module-based nested structure, weekly structure.
   - **Decision**: Module-based nested structure (4 modules + intro + hardware)
   - **Rationale**: Aligns with course learning outcomes (modules map to major topics: ROS 2, Simulation, Isaac, VLA), supports complex navigation with Docusaurus sidebar categories, allows code examples and diagrams to be co-located with chapters, scales better than flat structure for 20+ chapters
   - **Alternatives**: (1) Flat list (hard to navigate, no grouping), (2) Weekly structure (too granular, doesn't map to learning concepts)
   - **Consequences**: (+) Clear mental model, supports nested sidebar, modular content organization; (-) Requires more upfront planning, sidebar configuration more complex

4. **ADR: JSONB Columns for User Backgrounds**
   - **Context**: Need to store software and hardware background responses from questionnaire. Options: Separate tables (normalized), JSONB columns, TEXT columns with serialized JSON.
   - **Decision**: JSONB columns in `users` table
   - **Rationale**: Flexible schema (can add new questionnaire fields without migrations), PostgreSQL JSONB supports indexing and queries, simpler data model (no joins), aligns with agile hackathon timeline (easy to extend questionnaire)
   - **Alternatives**: (1) Normalized tables (user_programming_languages, user_hardware, etc.) - over-engineering for hackathon, (2) TEXT columns - no query support, manual serialization
   - **Consequences**: (+) Flexibility, simple queries, easy to extend; (-) Less strict validation (relies on application logic), harder to enforce constraints at DB level

### Reused ADRs from 001-hackathon-app

- **ADR: Docusaurus for Static Site Generation** (REUSE) - Still valid for Physical AI textbook
- **ADR: FastAPI for Backend API** (REUSE) - Still valid, extends with auth endpoints
- **ADR: RAG Strategy (Chunking, Embeddings, Retrieval)** (REUSE) - No changes, same approach for Physical AI content
- **ADR: Deployment Approach (GitHub Pages + Cloud Backend)** (REUSE) - Still valid

## Implementation Phases (High-Level)

### Phase 0: Research & Planning ✅ (Completed via /sp.plan)
- Research Physical AI content sources
- Research Neon Postgres integration patterns
- Research Better-auth setup with React/Docusaurus
- Generate `research.md`, `data-model.md`, `contracts/`, `quickstart.md`

### Phase 1: Neon Postgres & Better-auth Foundation (8-10 hours)
- Set up Neon Postgres account and database
- Create database schema (`users`, `conversations` tables)
- Implement `db_service.py` (connection pool, CRUD operations)
- Implement `auth_service.py` (signup, signin, JWT generation/validation)
- Create FastAPI auth endpoints (`/auth/signup`, `/auth/signin`, `/auth/me`)
- Extend `/ask` and `/ask_selected` endpoints to save conversations
- Write unit tests for database and auth services

### Phase 2: Physical AI Textbook Content Creation (16-20 hours)
- Write Introduction chapter (Physical AI overview, embodied intelligence)
- Write Module 1 chapters (ROS 2 architecture, nodes/topics, Python integration, URDF)
- Write Module 2 chapters (Gazebo simulation, URDF/SDF, physics, Unity rendering)
- Write Module 3 chapters (Isaac Sim, Isaac ROS, VSLAM, Nav2 planning)
- Write Module 4 chapters (VLA, Whisper voice-to-action, cognitive planning, capstone)
- Write Hardware Requirements chapter (workstation specs, Jetson kits, cameras, robots)
- Write Weekly Breakdown chapter (map 13 weeks to modules)
- Configure Docusaurus sidebar for nested module structure
- Add code examples and diagrams where appropriate

### Phase 3: Frontend Authentication UI (6-8 hours)
- Install Better-auth npm package in frontend
- Create `AuthProvider.tsx` (Better-auth context)
- Create `SignupForm.tsx` (email/password + software/hardware questionnaire)
- Create `SigninForm.tsx` (email/password)
- Create `ProfileButton.tsx` (display logged-in user, logout)
- Extend `Chatbot.tsx` to show user info and include JWT in API requests
- Create `/signup` and `/signin` pages in Docusaurus
- Style auth forms with custom CSS

### Phase 4: Integration & Testing (6-8 hours)
- Ingest Physical AI textbook content into Qdrant (`POST /ingest`)
- Test RAG pipeline with Physical AI questions
- Test selected-text mode with code examples from textbook
- Test signup flow (create user, save backgrounds to Neon Postgres)
- Test signin flow (authenticate, receive JWT)
- Test authenticated chatbot interactions (conversations saved with user_id)
- Test anonymous chatbot interactions (conversations saved with session_id)
- Test error scenarios (Qdrant down, Postgres down, invalid JWT)
- Verify agent skills work with Physical AI content

### Phase 5: Deployment & Demo Video (4-6 hours)
- Deploy textbook to GitHub Pages (`npm run build && npm run deploy`)
- Deploy FastAPI backend to cloud service (Render/Railway/Fly.io)
- Update frontend `config.js` with production API URL
- Verify end-to-end flow in production (signup → signin → ask question → see answer)
- Create 90-second demo video:
  - Show textbook navigation (modules, chapters)
  - Show chatbot asking Physical AI question with RAG
  - Show selected-text mode with code example
  - Show signup with background questionnaire
  - Show signin and authenticated chatbot
- Submit hackathon project (GitHub repo, published book, demo video)

**Total Estimated Effort**: 48-60 hours (within 72-hour hackathon timeline for full-day availability)

## Risk Analysis

### High-Priority Risks

1. **Risk: Textbook content creation takes longer than estimated**
   - **Impact**: Content is incomplete, missing modules or chapters
   - **Mitigation**: Prioritize Module 1 (ROS 2) and Module 3 (Isaac) as highest value; use AI assistance (Claude Code) to generate initial drafts; accept shorter chapters (600-800 words) if time-constrained
   - **Contingency**: Skip Module 4 (VLA) if necessary, focus on ROS 2 and Isaac as core content

2. **Risk: Neon Postgres integration issues (connection errors, query failures)**
   - **Impact**: Conversations not saved, authentication broken
   - **Mitigation**: Test Neon connection early in Phase 1; use connection pooling for reliability; implement retry logic for transient failures
   - **Contingency**: Fall back to SQLite for local testing, save Postgres integration for production deployment

3. **Risk: Better-auth integration complexity (JWT token management, CORS issues)**
   - **Impact**: Authentication doesn't work, users can't sign up/signin
   - **Mitigation**: Follow Better-auth documentation closely; test JWT flow with Postman before UI integration; configure CORS explicitly for auth endpoints
   - **Contingency**: Simplify to basic JWT auth without Better-auth library if integration fails

4. **Risk: Hackathon deadline pressure (72 hours)**
   - **Impact**: Incomplete implementation, missing bonus features
   - **Mitigation**: Follow strict prioritization (base 100 points first, then Better-auth for +50); use existing infrastructure (60% reuse); timebox each phase
   - **Contingency**: Skip Better-auth if behind schedule, focus on base 100 points + agent skills (already done) for 150 total

### Medium-Priority Risks

5. **Risk: RAG pipeline quality issues with Physical AI content (irrelevant chunks retrieved)**
   - **Impact**: Poor answer quality, users don't trust chatbot
   - **Mitigation**: Reuse proven RAG strategy from 001-hackathon-app (200-300 token chunks, top-5 retrieval); test with sample Physical AI questions early
   - **Contingency**: Adjust chunk size or retrieval count (top-3 or top-7) based on testing

6. **Risk: GitHub Pages deployment issues (build failures, routing problems)**
   - **Impact**: Textbook not accessible, can't submit for hackathon
   - **Mitigation**: Test local build (`npm run build`) early; follow Docusaurus deployment guide for GitHub Pages; configure baseUrl correctly
   - **Contingency**: Deploy to Vercel as alternative static hosting platform

### Low-Priority Risks

7. **Risk: Agent skills don't work well with Physical AI content (poor summaries, irrelevant quiz questions)**
   - **Impact**: Bonus points at risk
   - **Mitigation**: Agent skills already proven with educational content; test with Physical AI chapters early in Phase 4
   - **Contingency**: Document known limitations, emphasize P+Q+P pattern documentation for full credit

8. **Risk: Demo video creation complexity (recording, editing, time limit)**
   - **Impact**: Video exceeds 90 seconds, judges don't watch full demo
   - **Mitigation**: Script demo video before recording (30 seconds textbook nav, 30 seconds chatbot, 20 seconds auth, 10 seconds wrap-up); use NotebookLM for narration if available
   - **Contingency**: Record multiple takes, edit to exactly 90 seconds, prioritize showing base functionality over bonus features

## Success Metrics

### Hackathon Submission Criteria (Must-Have)

- ✅ Public GitHub repository with source code
- ✅ Published textbook accessible via GitHub Pages or Vercel URL
- ✅ Demo video under 90 seconds
- ✅ Textbook covers Physical AI & Humanoid Robotics course content (20+ chapters)
- ✅ RAG chatbot functional and embedded in textbook
- ✅ Selected-text mode implemented and tested
- ✅ Conversations saved to Neon Serverless Postgres
- ✅ Better-auth signup/signin with user background questionnaire
- ✅ Agent skills documented and functional

### Quality Metrics (Nice-to-Have)

- Textbook content is comprehensive and educational (manually reviewed for accuracy)
- RAG answers are accurate and cite relevant textbook sources (90%+ accuracy in testing)
- Conversations persist correctly in Neon Postgres (99%+ write success rate)
- Authentication flows work smoothly (no errors in testing)
- Demo video is engaging and showcases all key features (within 90 seconds)

### Point Breakdown (Target: 200)

- Base: Textbook + RAG + Selected-text + Neon Postgres = **100 points**
- Bonus: Better-auth + User background questionnaire = **+50 points**
- Bonus: Reusable agent skills (already implemented) = **+50 points**
- **Total Target: 200 points**

## Notes for Implementation

- **Content creation strategy**: Use AI assistance (Claude Code) to generate initial drafts of Physical AI chapters, then review and edit for accuracy and educational value. Prioritize breadth (cover all modules) over depth (shorter chapters acceptable).
- **Database migration**: Use raw SQL files in `backend/src/db/migrations/` for schema creation. Simple approach for hackathon; can migrate to Alembic for production.
- **Better-auth configuration**: Store JWT secret in environment variable; use httpOnly cookies for token storage in production; configure CORS to allow credentials.
- **Reuse strategy**: Copy entire `backend/` and `frontend/` structure from 001-hackathon-app branch, then extend with new files for Neon Postgres (db_service.py), Better-auth (auth_service.py), and Physical AI content (docs/).
- **Testing strategy**: Focus on integration tests for critical paths (signup → signin → ask question → verify conversation saved). Unit tests for database and auth services. Manual testing for UI flows.
- **Demo video script**: (1) Show textbook homepage with module navigation [15s], (2) Navigate to ROS 2 chapter and ask question via chatbot [20s], (3) Select text from NVIDIA Isaac chapter and ask question [15s], (4) Show signup form with background questionnaire [15s], (5) Sign in and show authenticated chatbot [15s], (6) Wrap up with project summary [10s]. Total: 90 seconds.
- **Deployment checklist**: (1) Build frontend (`npm run build`), (2) Deploy to GitHub Pages, (3) Deploy backend to Render/Railway with env vars, (4) Update frontend API URL, (5) Test end-to-end in production, (6) Record demo video, (7) Submit via hackathon form.
