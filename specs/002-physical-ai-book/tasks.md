# Task Breakdown: Physical AI & Humanoid Robotics Textbook

**Feature**: 002-physical-ai-book
**Branch**: `002-physical-ai-book`
**Created**: 2025-11-27
**Deadline**: Sunday, Nov 30, 2025 at 06:00 PM
**Target**: 300 points (Base 100 + Better-auth 50 + Personalization 50 + Urdu Translation 50 + Agent Skills 50)

---

## Task Organization Strategy

This task breakdown is organized by **User Story** to enable independent implementation and testing. Each phase corresponds to a user story from the specification, allowing parallel development where dependencies permit.

**User Story Priorities** (from spec.md):
- **US1 (P0)**: Access Educational Textbook on Physical AI
- **US2 (P0)**: Ask Questions Using RAG with Conversation Persistence
- **US3 (P1)**: Ask Questions About Selected Text
- **US4 (P1 - Bonus)**: Sign Up and Provide Background Information (+50 points)
- **US5 (P2 - Bonus)**: Use Reusable Agent Skills (+50 points, already implemented)

**Implementation Strategy**:
1. Complete Setup and Foundational phases first (blocking prerequisites)
2. Implement US1 (textbook content) and US2 (RAG + Neon Postgres) in parallel where possible
3. Add US3 (selected-text) and US4 (Better-auth) incrementally
4. US5 (agent skills) already complete from 001-hackathon-app, verify and document only

---

## Phase 1: Setup (Project Initialization)

**Goal**: Initialize project structure, configure development environment, set up external services

**Dependencies**: None (can start immediately)

**Tasks**:

- [x] T001 Checkout feature branch 002-physical-ai-book
- [x] T002 Create backend/src/db/ directory for database layer
- [x] T003 Create backend/src/db/migrations/ directory for SQL schema files
- [ ] T004 [P] Create Neon Serverless Postgres account at neon.tech (free tier)
- [ ] T005 [P] Create Qdrant Cloud account at cloud.qdrant.io if not exists (free tier, reuse from 001-hackathon-app)
- [ ] T006 [P] Verify OpenAI API key has sufficient quota for embeddings and chat completions
- [x] T007 Create backend/src/db/migrations/001_initial.sql with users and conversations tables per data-model.md
- [ ] T008 Run migration script on Neon Postgres to create schema (via Neon SQL Editor or psql)
- [x] T009 Update backend/requirements.txt to add psycopg2-binary, python-jose[cryptography], passlib[bcrypt]
- [x] T010 Update frontend/package.json to add better-auth npm package
- [x] T011 Create .env.example in backend/ with placeholders for NEON_DATABASE_URL, BETTER_AUTH_SECRET, OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY
- [ ] T012 Copy .env.example to .env and fill with actual credentials from Neon, Qdrant, OpenAI
- [ ] T013 Install backend dependencies: pip install -r backend/requirements.txt in activated venv
- [ ] T014 Install frontend dependencies: npm install in frontend/ directory

**Completion Criteria**:
- ✅ Branch checked out, all directories created
- ✅ External accounts created (Neon, Qdrant, OpenAI)
- ✅ Database schema deployed to Neon Postgres (users and conversations tables exist)
- ✅ Environment variables configured in .env (all required keys present)
- ✅ Dependencies installed (backend venv, frontend node_modules)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Goal**: Implement shared infrastructure required by multiple user stories (database layer, authentication base, RAG pipeline extension)

**Dependencies**: Phase 1 must be complete

**Tasks**:

- [x] T015 [P] Copy backend/src/models/ from 001-hackathon-app (chunk.py, question.py, answer.py, source_citation.py) - REUSE
- [x] T016 [P] Copy backend/src/services/rag_service.py from 001-hackathon-app - REUSE
- [x] T017 [P] Copy backend/src/services/ingestion_service.py from 001-hackathon-app - REUSE
- [x] T018 [P] Copy backend/src/services/skills/ directory from 001-hackathon-app (all 3 skills) - REUSE
- [x] T019 Create backend/src/models/user.py with User, UserCreate, UserLogin, SoftwareBackground, HardwareBackground Pydantic models per data-model.md
- [x] T020 Create backend/src/models/conversation.py with Conversation, ConversationCreate, Citation Pydantic models per data-model.md
- [x] T021 Create backend/src/db/connection.py with Neon Postgres connection pool using psycopg2.pool.SimpleConnectionPool (minconn=1, maxconn=10)
- [x] T022 Create backend/src/services/db_service.py with functions: init_db_pool(), get_db_connection(), release_db_connection(), create_user(), get_user_by_email(), save_conversation(), get_user_conversations()
- [x] T023 Create backend/src/services/auth_service.py with functions: hash_password(plain_password), verify_password(plain_password, hashed_password), create_access_token(user_id, email), verify_token(token)
- [x] T024 [P] Copy backend/src/config.py from 001-hackathon-app and extend to add NEON_DATABASE_URL and BETTER_AUTH_SECRET from environment variables
- [x] T025 [P] Copy backend/src/api/main.py from 001-hackathon-app and extend with lifespan event to initialize database connection pool on startup
- [x] T026 Create backend/src/api/dependencies.py with get_current_user(token: str) dependency for JWT authentication using auth_service.verify_token()
- [x] T027 [P] Copy frontend/src/components/Chatbot.tsx from 001-hackathon-app - REUSE (will extend in US2)
- [x] T028 [P] Copy frontend/src/components/SelectedTextHandler.tsx from 001-hackathon-app - REUSE (will use in US3)
- [x] T029 [P] Copy frontend/src/components/Chatbot.css from 001-hackathon-app - REUSE
- [x] T030 [P] Copy frontend/src/config.js from 001-hackathon-app and verify API_BASE_URL is set to http://localhost:8000

**Completion Criteria**:
- ✅ All reused code from 001-hackathon-app copied (models, services, components)
- ✅ New database models created (User, Conversation)
- ✅ Database connection pool implemented and tested (can connect to Neon Postgres)
- ✅ Auth service implemented (password hashing, JWT generation/validation work)
- ✅ FastAPI app extended with database initialization on startup

---

## Phase 3: User Story 1 - Access Educational Textbook (P0 - MVP)

**Story Goal**: Students can access comprehensive Physical AI textbook covering 13 weeks of course content (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA) via GitHub Pages with clear navigation

**Independent Test**: Deploy textbook to GitHub Pages, verify all 20+ chapters accessible, navigation works, content matches hackathon course requirements (ROS 2, Isaac, VLA, hardware specs)

**Dependencies**: Phase 2 complete (foundational setup done)

**Tasks**:

### T031-T036: Docusaurus Configuration

- [x] T031 [US1] Update frontend/docusaurus.config.js with title "Physical AI & Humanoid Robotics", tagline "13-Week Course on Embodied Intelligence", update baseUrl for GitHub Pages deployment
- [x] T032 [US1] Create frontend/sidebars.js with nested structure: intro, module1 (ROS 2), module2 (Gazebo/Unity), module3 (NVIDIA Isaac), module4 (VLA), hardware, weekly-breakdown
- [x] T033 [US1] Create frontend/docs/ directory structure: docs/intro.md, docs/module1/, docs/module2/, docs/module3/, docs/module4/, docs/hardware.md, docs/weekly-breakdown.md
- [x] T034 [US1] Update frontend/src/pages/index.js with Physical AI homepage hero section and feature highlights
- [x] T035 [US1] Create frontend/static/img/ directory and add placeholder logo.png for branding
- [x] T036 [US1] Create frontend/docs/intro.md with course introduction, learning objectives, prerequisites, course structure, and interactive features

### T037-T043: Introduction & Hardware Content

- [x] T037 [P] [US1] Write frontend/docs/intro.md (800-1,200 words): Introduction to Physical AI, embodied intelligence, transition from digital AI to physical robots, humanoid robotics landscape, course overview, learning outcomes per research.md
- [x] T038 [P] [US1] Write frontend/docs/hardware.md (1,000-1,500 words): Workstation requirements (RTX 4070 Ti+, Ubuntu 22.04, 64GB RAM, CPU specs), Jetson Orin kits (Nano 8GB, NX 16GB), RealSense cameras (D435i, D455), robot options (Unitree Go2, G1, alternatives), cloud alternatives (AWS RoboMaker, Omniverse Cloud), cost breakdowns per hackathon documentation
- [x] T039 [P] [US1] Write frontend/docs/weekly-breakdown.md (600-800 words): Map 13 weeks to modules (Weeks 1-2: Intro, 3-5: ROS 2, 6-7: Gazebo/Unity, 8-10: Isaac, 11-12: Humanoid Dev, 13: Conversational Robotics), topics per week, assessments (ROS 2 package, Gazebo sim, Isaac pipeline, capstone) per hackathon documentation
- [x] T040 [US1] Create frontend/docs/module1/index.md (400-600 words): Module 1 overview, introduce ROS 2 as robotic nervous system, list chapters, learning objectives (understand ROS 2 architecture, create nodes, implement pub/sub, define URDF)
- [x] T041 [US1] Create frontend/docs/module2/index.md (400-600 words): Module 2 overview, introduce Gazebo & Unity as digital twin, list chapters, learning objectives (simulate robots, understand URDF/SDF, test sensors)
- [x] T042 [US1] Create frontend/docs/module3/index.md (400-600 words): Module 3 overview, introduce NVIDIA Isaac as AI-robot brain, list chapters, learning objectives (use Isaac Sim, implement VSLAM, apply Nav2)
- [x] T043 [US1] Create frontend/docs/module4/index.md (400-600 words): Module 4 overview, introduce VLA (Vision-Language-Action), list chapters, learning objectives (integrate Whisper, use LLMs for planning, complete capstone)

### T044-T049: Module 1 Content (ROS 2)

- [x] T044 [P] [US1] Write frontend/docs/module1/ros2-architecture.md (800-1,200 words): ROS 2 architecture, DDS foundation, real-time communication, comparison to ROS 1, middleware role, code example (basic ROS 2 node setup) per research.md and hackathon documentation
- [x] T045 [P] [US1] Write frontend/docs/module1/nodes-topics-services.md (900-1,200 words): ROS 2 nodes (independent processes), topics (pub/sub for continuous data), services (request/response for discrete ops), actions (long-running tasks with feedback), code examples (publisher, subscriber, service client/server) per hackathon documentation
- [x] T046 [P] [US1] Write frontend/docs/module1/python-integration.md (800-1,100 words): rclpy (Python client library), creating nodes in Python, publishers and subscribers, launch files (XML/Python), parameter management, code example (complete Python node with rclpy) per hackathon documentation
- [x] T047 [P] [US1] Write frontend/docs/module1/urdf-for-humanoids.md (900-1,200 words): URDF (Unified Robot Description Format), links and joints, kinematic chains for humanoid robots, URDF example (simplified humanoid structure), integration with ROS 2, visualization in RViz per hackathon documentation

### T050-T053: Module 2 Content (Gazebo & Unity)

- [x] T050 [P] [US1] Write frontend/docs/module2/gazebo-simulation.md (800-1,200 words): Gazebo environment setup, simulating physics (gravity, collisions, friction), spawning robots, plugin architecture, integration with ROS 2, code example (launch file for Gazebo simulation) per hackathon documentation
- [x] T051 [P] [US1] Write frontend/docs/module2/urdf-vs-sdf.md (800-1,100 words): URDF vs SDF (Simulation Description Format), when to use each, robot description with visual and collision properties, inertial properties, code example (SDF for Gazebo-specific features) per hackathon documentation
- [x] T052 [P] [US1] Write frontend/docs/module2/physics-simulation.md (800-1,100 words): Rigid body dynamics, collision detection, friction models, gravity simulation, performance considerations, tuning physics parameters per hackathon documentation
- [x] T053 [P] [US1] Write frontend/docs/module2/unity-rendering.md (700-1,000 words): Unity for high-fidelity rendering, human-robot interaction scenarios, asset libraries, integration with ROS 2 (Unity Robotics Hub), photorealistic visualization vs Gazebo per hackathon documentation

### T054-T057: Module 3 Content (NVIDIA Isaac)

- [x] T054 [P] [US1] Write frontend/docs/module3/isaac-sim.md (900-1,200 words): NVIDIA Isaac Sim overview, Omniverse foundation, photorealistic simulation, synthetic data generation for training perception models, USD (Universal Scene Description) format, Isaac Sim vs Gazebo comparison, use cases for humanoid robots per hackathon documentation
- [ ] T055 [P] [US1] Write frontend/docs/module3/isaac-ros.md (900-1,200 words): Isaac ROS packages, hardware-accelerated perception (VSLAM, object detection, pose estimation), GPU acceleration benefits, integration with ROS 2, Isaac ROS GEMs (pre-built perception modules), code example (Isaac ROS VSLAM setup) per hackathon documentation
- [ ] T056 [P] [US1] Write frontend/docs/module3/vslam-navigation.md (900-1,200 words): VSLAM (Visual Simultaneous Localization and Mapping), camera-based navigation, map building while localizing, Isaac ROS VSLAM implementation, advantages for humanoid robots, comparison to LiDAR-based SLAM per hackathon documentation
- [ ] T057 [P] [US1] Write frontend/docs/module3/nav2-planning.md (900-1,200 words): Nav2 navigation stack, path planning algorithms, obstacle avoidance, goal-reaching in 2D/3D environments, bipedal locomotion considerations, Nav2 plugins, code example (Nav2 configuration for humanoid) per hackathon documentation

### T058-T062: Module 4 Content (VLA)

- [ ] T058 [P] [US1] Write frontend/docs/module4/voice-to-action.md (800-1,100 words): OpenAI Whisper for speech recognition, voice command processing, translating natural language to robot actions, integration with ROS 2 action servers, code example (Whisper + ROS 2 action client) per hackathon documentation
- [ ] T059 [P] [US1] Write frontend/docs/module4/cognitive-planning.md (900-1,200 words): Using LLMs (GPT-4) for high-level task planning, decomposing natural language instructions into ROS 2 action sequences, prompt engineering for robotics, example (LLM plans "Clean the room" into ROS 2 actions: navigate, detect objects, manipulate), limitations and safety considerations per hackathon documentation
- [ ] T060 [P] [US1] Write frontend/docs/module4/capstone-project.md (1,000-1,500 words): The Autonomous Humanoid capstone project, project description ("robot receives voice command, plans path, navigates obstacles, identifies object using computer vision, manipulates it"), integration of all modules (ROS 2 + Isaac Sim + VSLAM + Nav2 + Whisper + LLM), implementation steps, evaluation criteria, sample project structure per hackathon documentation
- [ ] T061 [US1] Review all textbook chapters for accuracy, completeness, internal consistency (cross-references between modules work, terminology consistent)
- [ ] T062 [US1] Test local textbook: npm start in frontend/, navigate to all modules/chapters, verify sidebar navigation, check code syntax highlighting, verify internal links work

**Completion Criteria (US1)**:
- ✅ 20+ chapters written (intro, hardware, weekly-breakdown, 4 module overviews, 16+ content chapters)
- ✅ All chapters 800-1,200 words each (comprehensive coverage)
- ✅ Content matches hackathon course requirements (ROS 2, Gazebo, Isaac, VLA, hardware specs)
- ✅ Docusaurus configuration complete (sidebar with nested modules)
- ✅ Local textbook builds without errors (npm run build succeeds)
- ✅ Navigation tested (can access any chapter from sidebar, internal links work)

---

## Phase 4: User Story 2 - RAG with Conversation Persistence (P0 - MVP)

**Story Goal**: Students can ask questions about textbook content and receive accurate RAG-based answers with source citations. Conversations are persisted to Neon Serverless Postgres for history tracking.

**Independent Test**: Ask questions via chatbot, verify answers come from textbook context, check source citations, query Neon Postgres to confirm conversations saved with question, answer, citations, timestamp, user_id (if authenticated) or session_id (if anonymous)

**Dependencies**: Phase 2 (foundational), US1 (textbook content written for ingestion)

**Tasks**:

### T063-T067: Backend RAG with Neon Postgres

- [x] T063 [P] [US2] Copy backend/src/api/endpoints/ingest.py from 001-hackathon-app - REUSE (no changes needed, uses ingestion_service.py)
- [x] T064 [P] [US2] Copy backend/src/api/endpoints/ping.py from 001-hackathon-app - REUSE (health check, no changes)
- [x] T065 [US2] Copy backend/src/api/endpoints/ask.py from 001-hackathon-app and EXTEND: after generating answer, call db_service.save_conversation(user_id, session_id, question, answer, citations, 'rag') to persist to Neon Postgres, include user_id from JWT if authenticated (get_current_user dependency, optional), include session_id from request if anonymous
- [x] T066 [US2] Update backend/src/api/main.py to register ask and ingest routers, verify lifespan event initializes database connection pool
- [ ] T067 [US2] Test RAG backend: Start uvicorn, POST to /ingest (ingest textbook content), POST to /ask with Physical AI question, verify answer returned with source citations, check Neon Postgres conversations table (SELECT * FROM conversations ORDER BY timestamp DESC LIMIT 1) to confirm conversation saved

### T068-T071: Frontend Chatbot Integration

- [x] T068 [US2] Extend frontend/src/components/Chatbot.tsx: Update to handle both authenticated and anonymous users (if JWT token in localStorage, include in Authorization header; if no token, generate session_id UUID and include in request), extract conversation_id from response
- [x] T069 [US2] Embed Chatbot component in Docusaurus layout: Update frontend/src/theme/DocItem/Layout/index.js (or equivalent) to import and render <Chatbot /> component at bottom of each textbook page
- [x] T070 [US2] Update frontend/src/config.js to ensure API_BASE_URL points to backend (http://localhost:8000 for dev, production URL for deployment)
- [ ] T071 [US2] Test chatbot in browser: npm start, navigate to any chapter, use embedded chatbot to ask "What is Physical AI?", verify answer appears with source citations, check browser network tab to confirm POST /ask succeeded, check Neon Postgres to verify conversation saved

**Completion Criteria (US2)**:
- ✅ Textbook content ingested into Qdrant (POST /ingest succeeds, chunks stored)
- ✅ RAG pipeline functional (POST /ask returns accurate answers with source citations from textbook)
- ✅ Conversations persisted to Neon Postgres (conversations table populated with question, answer, citations JSON, timestamp)
- ✅ Chatbot embedded in textbook pages (visible at bottom of each chapter)
- ✅ Both authenticated and anonymous modes work (user_id or session_id stored correctly)

---

## Phase 5: User Story 3 - Selected Text Mode (P1)

**Story Goal**: Students can select text in the textbook, click "Ask about this selection," and receive answers based only on that selected text (no RAG query)

**Independent Test**: Select text from any chapter, verify "Ask about this selection" button appears, click button, ask question, verify answer uses only selected text, check Neon Postgres to confirm conversation saved with question_type='selected_text'

**Dependencies**: Phase 2 (foundational), US2 (chatbot working, Neon Postgres integrated)

**Tasks**:

- [x] T072 [P] [US3] Copy backend/src/api/endpoints/ask_selected.py from 001-hackathon-app and EXTEND: after generating answer, call db_service.save_conversation(user_id, session_id, question, answer, [{"source_type": "user_selection", "text_excerpt": context[:200]}], 'selected_text') to persist to Neon Postgres
- [x] T073 [US3] Update backend/src/api/main.py to register ask_selected router
- [ ] T074 [US3] Test ask_selected endpoint: POST to /ask_selected with {"question": "Explain this", "context": "ROS 2 is a middleware framework..."}, verify answer uses only provided context, check Neon Postgres (SELECT * FROM conversations WHERE question_type='selected_text') to confirm saved
- [x] T075 [US3] Verify frontend/src/components/SelectedTextHandler.tsx from Phase 2 (already copied from 001-hackathon-app), ensure it triggers on text selection (mouseup, keyup events), shows "Ask about this selection" button when minimum 10 characters selected
- [x] T076 [US3] Embed SelectedTextHandler in Docusaurus layout: Update frontend/src/theme/DocItem/Layout/index.js to import and render <SelectedTextHandler /> component alongside Chatbot
- [ ] T077 [US3] Test selected-text mode in browser: Select paragraph from any chapter, verify button appears, click button, chatbot opens with selected text, ask question, verify answer based only on selected text

**Completion Criteria (US3)**:
- ✅ Selected-text endpoint functional (POST /ask_selected returns answers using only provided context)
- ✅ SelectedTextHandler component active (button appears on text selection)
- ✅ Conversations saved with question_type='selected_text' (distinct from RAG queries in database)
- ✅ End-to-end flow tested (select text → button → ask → answer → verify in Neon Postgres)

---

## Phase 6: User Story 4 - Better-auth with User Background (P1 - Bonus +50)

**Story Goal**: Students can sign up with email/password and provide software/hardware background information. Returning students can sign in. User profiles stored in Neon Postgres with backgrounds in JSONB columns.

**Independent Test**: Sign up with test account, complete background questionnaire (software: programming languages, robotics experience, AI/ML level; hardware: RTX GPU, Jetson, robot access), verify profile stored in Neon Postgres users table, sign in with same credentials, verify JWT token received and authenticated chatbot interactions save user_id

**Dependencies**: Phase 2 (foundational: auth_service, db_service), US2 (chatbot integrated for authenticated flow)

**Tasks**:

### T078-T082: Backend Authentication Endpoints

- [x] T078 [P] [US4] Create backend/src/api/endpoints/auth.py with POST /auth/signup endpoint: accept SignupRequest (email, password, software_background, hardware_background), validate email format and uniqueness (db_service.get_user_by_email returns None), hash password with auth_service.hash_password(), call db_service.create_user() to insert into Neon Postgres users table, generate JWT token with auth_service.create_access_token(user_id, email), return AuthResponse {"user_id", "email", "token"}
- [x] T079 [P] [US4] Add POST /auth/signin endpoint to auth.py: accept {"email", "password"}, query db_service.get_user_by_email(), verify password with auth_service.verify_password(), if valid update last_login timestamp, generate JWT token, return AuthResponse {"user_id", "email", "token"}
- [x] T080 [P] [US4] Add GET /auth/me endpoint to auth.py (protected by get_current_user dependency): extract user_id from JWT token, query db_service.get_user_by_id(), return UserProfile {"user_id", "email", "software_background", "hardware_background", "created_at", "last_login"}
- [x] T081 [US4] Update backend/src/api/main.py to register auth router
- [ ] T082 [US4] Test auth endpoints: POST /auth/signup with test data, verify user created in Neon Postgres (SELECT * FROM users WHERE email='test@example.com'), POST /auth/signin with same credentials, verify JWT token returned, GET /auth/me with Authorization header (Bearer <token>), verify profile returned

### T083-T088: Frontend Authentication UI

- [x] T083 [P] [US4] Install better-auth in frontend: npm install better-auth (already added to package.json in Phase 1)
- [x] T084 [US4] Create frontend/src/lib/auth.ts with createAuthClient configured for API_BASE_URL (http://localhost:8000), credentials: 'include'
- [x] T085 [US4] Create frontend/src/components/SignupForm.tsx: Form with fields (email, password, software_background: multi-select for programming languages, dropdown for robotics_experience [none/beginner/intermediate/advanced], dropdown for ai_ml_level [none/basic/intermediate/advanced]; hardware_background: checkbox for rtx_gpu_access, text input for rtx_gpu_model, dropdown for jetson_kit, dropdown for robot_hardware), on submit POST to /auth/signup, store JWT token in localStorage, redirect to textbook homepage
- [x] T086 [US4] Create frontend/src/components/SigninForm.tsx: Form with email and password fields, on submit POST to /auth/signin, store JWT token in localStorage, redirect to textbook
- [x] T087 [US4] Create frontend/src/components/AuthProvider.tsx: React context provider to manage authentication state (isAuthenticated, user profile), check localStorage for token on mount, provide signOut function (clear token, redirect to signin)
- [x] T088 [US4] Create frontend/src/pages/signup.tsx and frontend/src/pages/signin.tsx: Render SignupForm and SigninForm components respectively

### T089-T091: Frontend Integration

- [x] T089 [US4] Update frontend/src/components/Chatbot.tsx to check for JWT token in localStorage, if present include in Authorization header for POST /ask and POST /ask_selected requests (enables user_id tracking in Neon Postgres conversations)
- [x] T090 [US4] Create frontend/src/components/ProfileButton.tsx: Display logged-in user email (from AuthProvider context), show "Sign Out" button (calls authContext.signOut()), render in Docusaurus navbar
- [x] T091 [US4] Update Docusaurus navbar configuration in frontend/docusaurus.config.js to include links to /signup and /signin pages, conditionally show ProfileButton when authenticated

### T092-T093: End-to-End Testing

- [ ] T092 [US4] Test signup flow: Navigate to /signup, fill out form with test data (email: test@example.com, password: password123, software_background: Python/C++/intermediate/basic, hardware_background: RTX 4070 Ti/Yes/Orin Nano 8GB/none), submit, verify redirected to textbook, verify token in localStorage, check Neon Postgres users table (SELECT email, software_background, hardware_background FROM users WHERE email='test@example.com')
- [ ] T093 [US4] Test signin flow: Sign out, navigate to /signin, enter test credentials, submit, verify authenticated, ask question via chatbot, check Neon Postgres conversations table (SELECT user_id, question FROM conversations ORDER BY timestamp DESC LIMIT 1), verify user_id matches test user's id (not NULL)

**Completion Criteria (US4)**:
- ✅ Signup endpoint functional (POST /auth/signup creates user in Neon Postgres, returns JWT)
- ✅ Signin endpoint functional (POST /auth/signin validates credentials, returns JWT)
- ✅ User background questionnaire collects software and hardware information (stored as JSONB in users table)
- ✅ Authenticated chatbot interactions save user_id (conversations.user_id populated for logged-in users)
- ✅ Signup/signin UI complete and integrated with Docusaurus navbar
- ✅ End-to-end auth flow tested (signup → signin → authenticated chatbot → verify database)

---

## Phase 7: Content Personalization (Bonus +50)

**Story Goal**: Logged-in students can personalize textbook content based on their software/hardware background by clicking a "Personalize for Me" button at the start of each chapter. System uses user background data (from Neon Postgres) to dynamically adjust content complexity level.

**Independent Test**: Sign in as user with specific background (e.g., beginner robotics experience, no RTX GPU), navigate to chapter, click "Personalize for Me" button, verify content is adjusted for beginner level with cloud-based alternatives for hardware requirements, check personalization request saved to user preferences

**Dependencies**: Phase 6 (US4 - Better-auth complete, user backgrounds stored in Neon Postgres)

**Tasks**:

### T094-T098: Backend Personalization Service

- [ ] T094 [P] [BONUS] Create backend/src/services/personalization_service.py with function analyze_user_background(user_id): Query Neon Postgres users table for user's software_background and hardware_background JSONB, determine complexity level (beginner/intermediate/advanced) based on robotics_experience and ai_ml_level
- [ ] T095 [P] [BONUS] Add function personalize_content(chapter_content, user_background, complexity_level) to personalization_service.py: Use OpenAI ChatCompletion API to rewrite chapter content adjusting for complexity level (beginner: simpler language, more explanations; advanced: concise, assume prior knowledge), include hardware alternatives if user lacks RTX GPU/Jetson (suggest cloud options from hackathon docs)
- [ ] T096 [BONUS] Create backend/src/models/personalization.py with PersonalizeRequest (chapter_id, user_id) and PersonalizeResponse (personalized_content, complexity_level, hardware_alternatives) Pydantic models
- [ ] T097 [BONUS] Create backend/src/api/endpoints/personalize.py with POST /personalize endpoint (protected by get_current_user dependency): Accept PersonalizeRequest, call personalization_service.analyze_user_background(), fetch original chapter content, call personalization_service.personalize_content(), return PersonalizeResponse with adjusted content
- [ ] T098 [BONUS] Update backend/src/api/main.py to register personalize router

### T099-T103: Frontend Personalization UI

- [ ] T099 [P] [BONUS] Create frontend/src/components/PersonalizeButton.tsx: React component with "Personalize for Me" button, onClick handler calls POST /personalize with current chapter_id and JWT token, displays loading state during API call, replaces chapter content with personalized version on success
- [ ] T100 [P] [BONUS] Create frontend/src/hooks/usePersonalization.ts: Custom React hook managing personalization state (isPersonalized, personalizedContent, loading, error), provides togglePersonalization() function to switch between original and personalized content
- [ ] T101 [BONUS] Update frontend/src/theme/DocItem/Layout/index.js to embed PersonalizeButton component at top of each chapter (after title, before content), conditionally render only if user is authenticated (check AuthProvider context)
- [ ] T102 [BONUS] Update frontend/src/css/custom.css to add styles for PersonalizeButton (prominent placement, clear "Personalize for Me" label, loading spinner, success/error states)
- [ ] T103 [BONUS] Add user preference storage in frontend: Create frontend/src/utils/localStorage.ts with functions to save/load personalization preferences per chapter (e.g., user prefers personalized version of Module 1 Chapter 2), restore preference on page load

### T104-T105: Personalization Testing

- [ ] T104 [BONUS] Test personalization backend: Sign up test user with beginner robotics_experience and no RTX GPU, POST to /personalize with Module 1 ROS 2 chapter, verify response contains simplified content with beginner-friendly explanations and cloud alternative suggestions (AWS RoboMaker instead of local RTX setup)
- [ ] T105 [BONUS] Test personalization end-to-end: Sign in as beginner user, navigate to NVIDIA Isaac chapter, click "Personalize for Me" button, verify content adjusted to explain Isaac Sim concepts in simpler terms with Omniverse Cloud alternative mentioned, click button again to toggle back to original content, verify localStorage stores preference

**Completion Criteria (Personalization +50)**:
- ✅ Personalization service analyzes user background from Neon Postgres JSONB columns
- ✅ Content dynamically adjusted for complexity level (beginner/intermediate/advanced)
- ✅ Hardware alternatives provided for users without RTX GPU/Jetson (cloud options from hackathon docs)
- ✅ "Personalize for Me" button appears at start of each chapter for authenticated users
- ✅ Personalization toggle works (switch between original and personalized content)
- ✅ User preferences persisted in localStorage (remembers personalization choice per chapter)

---

## Phase 8: Urdu Translation (Bonus +50)

**Story Goal**: Logged-in students can translate textbook content to Urdu by clicking a "Translate to Urdu" button at the start of each chapter. System uses translation API (OpenAI or Google Translate) to convert English content to Urdu with proper RTL (right-to-left) text rendering.

**Independent Test**: Sign in as any user, navigate to any chapter, click "Translate to Urdu" button, verify content translated to Urdu with RTL text rendering, check translation cached to reduce API costs, toggle back to English, verify localStorage stores language preference

**Dependencies**: Phase 6 (US4 - Better-auth complete for authenticated users), Phase 7 (Personalization infrastructure can be reused for translation state management)

**Tasks**:

### T106-T110: Backend Translation Service

- [ ] T106 [P] [BONUS] Create backend/src/services/translation_service.py with function translate_to_urdu(english_text): Use OpenAI ChatCompletion API with system prompt "Translate the following technical content to Urdu, preserving technical terms (ROS 2, URDF, NVIDIA Isaac) in English" or use Google Translate API (googletrans library), return translated Urdu text
- [ ] T107 [P] [BONUS] Add translation caching to translation_service.py: Create backend/src/db/migrations/002_translations.sql with translations table (id, chapter_id, language, translated_content TEXT, created_at), store translations in Neon Postgres to avoid re-translating same content, check cache before calling translation API
- [ ] T108 [BONUS] Run migration 002_translations.sql on Neon Postgres to create translations table
- [ ] T109 [BONUS] Create backend/src/api/endpoints/translate.py with POST /translate endpoint (protected by get_current_user dependency): Accept TranslateRequest (chapter_id, target_language="urdu"), check translation_service cache, if not cached call translate_to_urdu(), store in cache, return TranslateResponse (translated_content, language, cached=true/false)
- [ ] T110 [BONUS] Update backend/src/api/main.py to register translate router

### T111-T116: Frontend Translation UI

- [ ] T111 [P] [BONUS] Create frontend/src/components/TranslateButton.tsx: React component with "Translate to Urdu" / "Show Original (English)" toggle button, onClick handler calls POST /translate with current chapter_id and target_language="urdu", displays loading state, replaces chapter content with Urdu translation on success
- [ ] T112 [P] [BONUS] Create frontend/src/hooks/useTranslation.ts: Custom React hook managing translation state (currentLanguage, translatedContent, loading, error), provides toggleLanguage(target_language) function, integrates with localStorage to persist language preference per chapter
- [ ] T113 [BONUS] Update frontend/src/theme/DocItem/Layout/index.js to embed TranslateButton component at top of each chapter (next to PersonalizeButton if both features enabled), conditionally render only if user is authenticated
- [ ] T114 [BONUS] Add RTL text rendering support in frontend/src/css/custom.css: Add CSS class .rtl-text with direction: rtl, text-align: right, apply to chapter content when language is Urdu, ensure code blocks and diagrams remain LTR (direction: ltr for pre/code elements)
- [ ] T115 [BONUS] Update frontend/docusaurus.config.js to add Urdu (ur) to i18n configuration if needed for proper locale handling, ensure fonts support Urdu script (use Google Fonts Noto Nastaliq Urdu or similar)
- [ ] T116 [BONUS] Add language preference storage: Update frontend/src/utils/localStorage.ts to save/load translation preferences per chapter (e.g., user prefers Urdu for Module 2), restore preference on page load

### T117-T118: Translation Testing

- [ ] T117 [BONUS] Test translation backend: POST to /translate with Module 1 ROS 2 chapter and target_language="urdu", verify response contains Urdu text with technical terms (ROS 2, URDF) preserved in English, check Neon Postgres translations table (SELECT chapter_id, language FROM translations WHERE language='urdu'), verify translation cached
- [ ] T118 [BONUS] Test translation end-to-end: Sign in, navigate to Gazebo chapter, click "Translate to Urdu" button, verify content translated to Urdu with RTL rendering (text flows right-to-left), verify code examples remain LTR, click button again to toggle back to English, verify localStorage stores language preference, reload page and verify Urdu version loads automatically

**Completion Criteria (Urdu Translation +50)**:
- ✅ Translation service converts English content to Urdu using OpenAI or Google Translate API
- ✅ Technical terms preserved in English within Urdu translation (ROS 2, NVIDIA Isaac, URDF, etc.)
- ✅ "Translate to Urdu" button appears at start of each chapter for authenticated users
- ✅ RTL (right-to-left) text rendering works correctly for Urdu content
- ✅ Code blocks and diagrams remain LTR (left-to-right) for readability
- ✅ Translations cached in Neon Postgres to reduce API costs (avoid re-translating same content)
- ✅ Language preference persisted in localStorage (remembers Urdu/English choice per chapter)
- ✅ Toggle between English and Urdu works seamlessly

---

## Phase 9: User Story 5 - Verify Reusable Agent Skills (P2 - Bonus +50)

**Story Goal**: Confirm reusable agent skills (SummarizeSection, GenerateQuizQuestions, ExplainTerm) work with Physical AI textbook content and are documented with P+Q+P pattern

**Independent Test**: Call each skill endpoint with Physical AI content, verify outputs are educationally valuable, check skill documentation includes persona, analytical questions, decision principles

**Dependencies**: Phase 2 (foundational: skills copied from 001-hackathon-app)

**Tasks**:

- [ ] T094 [P] [US5] Copy backend/src/api/endpoints/skills.py from 001-hackathon-app - REUSE (endpoints for /skills/summarize, /skills/quiz, /skills/explain)
- [ ] T095 [US5] Update backend/src/api/main.py to register skills router if not already done
- [ ] T096 [US5] Test SummarizeSection skill: POST to /skills/summarize with text from Module 1 (ROS 2 chapter), verify summary is concise and preserves key concepts (nodes, topics, services)
- [ ] T097 [US5] Test GenerateQuizQuestions skill: POST to /skills/quiz with text from Module 3 (NVIDIA Isaac chapter), verify questions are multiple-choice with plausible distractors and explanations
- [ ] T098 [US5] Test ExplainTerm skill: POST to /skills/explain with term "VSLAM", verify explanation uses RAG to find relevant textbook context (Isaac ROS chapter) and provides clear definition
- [ ] T099 [US5] Review backend/src/services/skills/ directory: Verify each skill file (summarize_section.py, generate_quiz.py, explain_term.py) includes documentation with Persona, Analytical Questions, Decision Principles (P+Q+P pattern) as inline comments or docstrings

**Completion Criteria (US5)**:
- ✅ All 3 agent skills functional and tested with Physical AI content
- ✅ SummarizeSection produces concise, accurate summaries (20-30% of original length)
- ✅ GenerateQuizQuestions creates valid multiple-choice questions with correct answers
- ✅ ExplainTerm integrates with RAG to provide context-aware explanations
- ✅ P+Q+P pattern documented for all skills (persona, questions, principles visible in code)

---

## Phase 8: Polish & Cross-Cutting Concerns

**Goal**: Finalize deployment configuration, create demo video, perform final testing, prepare for hackathon submission

**Dependencies**: All user stories (US1-US5) complete

**Tasks**:

### T100-T104: Architecture Decision Records (ADRs)

- [ ] T100 [P] Create history/adr/neon-postgres-choice.md: Document decision to use Neon Serverless Postgres, alternatives (local PostgreSQL, Supabase, SQLite), rationale (serverless, free tier, JSONB support, FastAPI integration), consequences per plan.md
- [ ] T101 [P] Create history/adr/better-auth-integration.md: Document decision to use Better-auth, alternatives (custom JWT, NextAuth, Supabase Auth), rationale (hackathon requirement, TypeScript-first, lightweight, JWT built-in), consequences per plan.md
- [ ] T102 [P] Create history/adr/textbook-content-structure.md: Document decision for nested module directories, alternatives (flat structure, weekly structure), rationale (scalable, clear mental model, Docusaurus sidebar support), consequences per plan.md
- [ ] T103 [P] Create history/adr/jsonb-user-backgrounds.md: Document decision to use JSONB columns for user backgrounds, alternatives (normalized tables, TEXT columns), rationale (flexibility, agile development, PostgreSQL strength, easy to extend), consequences per plan.md
- [ ] T104 Update history/adr/README.md to reference 4 new ADRs and 4 reused ADRs from 001-hackathon-app (Docusaurus, FastAPI, RAG strategy, deployment)

### T105-T110: Deployment Preparation

- [ ] T105 Update frontend/docusaurus.config.js with production baseUrl (GitHub Pages URL: https://<username>.github.io/<repo>/)
- [ ] T106 Build frontend for production: npm run build in frontend/, verify build/ directory created without errors
- [ ] T107 Test production build locally: npm run serve in frontend/, navigate to http://localhost:3000, test all features (navigation, chatbot, selected-text, auth)
- [ ] T108 Deploy frontend to GitHub Pages: npm run deploy (pushes to gh-pages branch), wait for GitHub Actions to complete, verify textbook accessible at production URL
- [ ] T109 Deploy backend to cloud service (Render/Railway/Fly.io): Create new web service, connect GitHub repo, set build/start commands (pip install -r requirements.txt, uvicorn src.api.main:app --host 0.0.0.0 --port $PORT), add environment variables (OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_DATABASE_URL, BETTER_AUTH_SECRET), deploy
- [ ] T110 Update frontend/src/config.js with production backend URL (e.g., https://physical-ai-backend.onrender.com), rebuild and redeploy frontend

### T111-T114: Final Testing

- [ ] T111 Test end-to-end flow in production: Visit deployed textbook, navigate modules, sign up with new account, fill background questionnaire, sign in, ask RAG question via chatbot, select text and ask question, test agent skills (if exposed in UI or via API docs), verify all features work
- [ ] T112 Verify Neon Postgres data: Connect to production Neon database, query users table (SELECT COUNT(*) FROM users), query conversations table (SELECT COUNT(*) FROM conversations WHERE question_type='rag' AND question_type='selected_text'), confirm data persisted correctly
- [ ] T113 Performance testing: Use browser DevTools Network tab to measure page load times (should be < 2 seconds for textbook pages), RAG query response times (should be < 7 seconds p95), auth operations (signup/signin < 3 seconds)
- [ ] T114 Cross-browser testing: Test on Chrome, Firefox, Safari, Edge (verify chatbot, auth forms, selected-text button work on all browsers)

### T115-T117: Demo Video Creation

- [ ] T115 Script demo video (90 seconds max): [0-15s] Show textbook homepage with module navigation, [15-35s] Navigate to ROS 2 chapter and ask question via chatbot with RAG answer and citations, [35-50s] Select text from NVIDIA Isaac chapter and ask question with selected-text answer, [50-70s] Show signup form with background questionnaire, [70-85s] Sign in and show authenticated chatbot with user profile, [85-90s] Wrap up with project summary and point total (200 points: base 100 + Better-auth 50 + agent skills 50)
- [ ] T116 Record demo video: Use screen recording tool (OBS Studio, QuickTime, built-in browser recorder), follow script, ensure audio is clear (use NotebookLM for narration if available), keep video under 90 seconds (judges only watch first 90 seconds)
- [ ] T117 Edit and upload demo video: Trim to exactly 90 seconds, add title card with project name "Physical AI & Humanoid Robotics Textbook" and GitHub repo URL, upload to YouTube (unlisted), copy video link for submission form

### T118-T120: Hackathon Submission

- [ ] T118 Create final README.md in repository root: Include project description, features implemented (textbook with 20+ chapters, RAG chatbot, selected-text mode, Better-auth, agent skills), tech stack, setup instructions (link to quickstart.md), deployment URLs (GitHub Pages book, backend API), demo video link, point breakdown (200 points: base 100 + Better-auth 50 + skills 50), screenshots
- [ ] T119 Prepare submission materials: Public GitHub repository URL, published textbook URL (GitHub Pages), demo video link (YouTube), WhatsApp number for live presentation invitation
- [ ] T120 Submit hackathon project via form: Navigate to https://forms.gle/CQsSEGM3GeCrL43c8, fill in GitHub repo link, published book link, demo video link, WhatsApp number, submit before Sunday Nov 30, 2025 at 06:00 PM deadline

**Completion Criteria (Phase 8)**:
- ✅ 4 new ADRs created and documented (Neon Postgres, Better-auth, textbook structure, JSONB backgrounds)
- ✅ Frontend deployed to GitHub Pages and accessible via public URL
- ✅ Backend deployed to cloud service and accessible from frontend
- ✅ End-to-end testing complete in production environment
- ✅ Demo video created (under 90 seconds, covers all features)
- ✅ Hackathon submission completed (form submitted with all required links)

---

## Task Summary

**Total Tasks**: 145 tasks across 10 phases

**Task Breakdown by Phase**:
- Phase 1 (Setup): 14 tasks
- Phase 2 (Foundational): 16 tasks
- Phase 3 (US1 - Textbook Content): 32 tasks
- Phase 4 (US2 - RAG + Neon Postgres): 9 tasks
- Phase 5 (US3 - Selected Text): 6 tasks
- Phase 6 (US4 - Better-auth): 16 tasks
- Phase 7 (Content Personalization - Bonus): 12 tasks
- Phase 8 (Urdu Translation - Bonus): 13 tasks
- Phase 9 (US5 - Agent Skills): 6 tasks
- Phase 10 (Polish & Deployment): 21 tasks

**Parallel Opportunities**:
- Phase 1: T004, T005, T006 (account creation), T009, T010 (requirements updates) can run in parallel
- Phase 2: T015-T018 (copying from 001-hackathon-app), T019-T020 (new models), T024, T027-T030 (frontend copies) can run in parallel
- Phase 3: Most content writing tasks (T037-T060) can run in parallel (different markdown files, no dependencies)
- Phase 6: T078-T080 (backend endpoints), T083-T088 (frontend UI) can run in parallel
- Phase 8: T100-T104 (ADRs), T105-T110 (deployment) can run partially in parallel

**Critical Path** (longest dependency chain):
Setup (Phase 1) → Foundational (Phase 2) → US1 Content Writing (Phase 3, 32 tasks, ~16-20 hours) → US2 Integration (Phase 4) → US4 Better-auth (Phase 6) → Final Testing & Deployment (Phase 8)

**Estimated Total Effort**: 48-60 hours for solo developer (matches plan.md timeline)

---

## Dependencies & Execution Order

**Dependency Graph**:
```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational)
    ↓
    ├─→ Phase 3 (US1 - Textbook Content) ←─┐
    │                                        │
    ├─→ Phase 4 (US2 - RAG + Postgres) ──→ Required for US5
    │        ↓                               │
    ├─→ Phase 5 (US3 - Selected Text)       │
    │        ↓                               │
    └─→ Phase 6 (US4 - Better-auth) ←──────┘
             ↓
        Phase 7 (US5 - Agent Skills Verification)
             ↓
        Phase 8 (Polish & Deployment)
```

**Execution Strategy**:
1. Complete Phase 1 and Phase 2 sequentially (blocking prerequisites)
2. Start Phase 3 (textbook content writing) immediately after Phase 2
3. While Phase 3 is in progress (long-running), start Phase 4 (RAG integration) in parallel
4. After Phase 4 complete, add Phase 5 (selected-text) and Phase 6 (Better-auth) incrementally
5. Phase 7 (verify agent skills) is quick validation, can run anytime after Phase 2
6. Phase 8 (polish & deployment) requires all previous phases complete

**MVP Scope** (Minimum for 100 base points):
- Phase 1: Setup
- Phase 2: Foundational
- Phase 3: US1 (Textbook Content) - can be shortened to ~10 core chapters if time-constrained
- Phase 4: US2 (RAG + Neon Postgres)
- Phase 5: US3 (Selected Text)
- Phase 10: Deployment & Demo Video

**Full Scope** (For 300 points - ALL bonuses):
- Add Phase 6: US4 (Better-auth) for +50 points
- Add Phase 7: Content Personalization for +50 points
- Add Phase 8: Urdu Translation for +50 points
- Add Phase 9: US5 (Agent Skills) for +50 points (minimal effort, already done from 001-hackathon-app)

---

## Notes

- **Content Creation Priority**: Phase 3 (US1) is the longest phase (32 tasks, ~16-20 hours). Use AI assistance (Claude Code) to generate initial drafts for each chapter, then review and edit for accuracy and alignment with hackathon documentation.
- **Hackathon Documentation Compliance**: All textbook content tasks (T037-T060) explicitly reference "per hackathon documentation" to ensure strict adherence to course requirements (ROS 2, Gazebo, NVIDIA Isaac, VLA, hardware specs, assessments, capstone project).
- **Reuse Strategy**: 60% of infrastructure already complete from 001-hackathon-app (RAG pipeline, chatbot, agent skills). Focus new effort on Physical AI content, Neon Postgres integration, and Better-auth.
- **Testing Approach**: Each phase includes explicit test tasks (e.g., T067 test RAG backend, T071 test chatbot, T082 test auth endpoints). Manual testing is primary approach for frontend features (auth flows, chatbot interactions).
- **Parallel Execution**: Where marked with [P], tasks can run in parallel (different files, no blocking dependencies). Maximize parallelization in content writing phase (T037-T060).
- **Format Compliance**: All tasks follow strict checklist format (checkbox, Task ID, [P] if parallel, [Story] label for user story phases, description with file path).
