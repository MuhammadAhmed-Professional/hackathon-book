<!--
Sync Impact Report:
- Version change: 1.0.1 → 2.0.0 (MAJOR - complete scope redefinition from generic AI-Driven Development book to Physical AI & Humanoid Robotics hackathon textbook)
- Modified principles:
  - Section II: Expanded to include hackathon-specific reusable intelligence (agent skills, PHRs for Physical AI content patterns)
  - Section IV: Updated educational focus from generic students to Physical AI course learners (robotics, ROS 2, simulation, hardware)
- Added sections:
  - Physical AI Textbook Standards (Modules 1-4, hardware requirements, 13-week course structure, assessments)
  - Content Personalization Standards (user background, complexity adjustment, OpenAI integration)
  - Urdu Translation Standards (RTL rendering, caching, terminology consistency)
  - Neon Serverless Postgres Requirements (conversations, user backgrounds, translations tables)
  - Better-auth Integration Requirements (signup/signin, JWT, user background questionnaire)
- Removed sections:
  - Generic "AI-Driven and Spec-Driven Development" book topic references
  - Generic book chapter organization (replaced with Physical AI modules)
- Templates requiring updates:
  - ✅ spec-template.md (already aligned - 002-physical-ai-book spec follows constitution)
  - ✅ plan-template.md (already aligned - 002-physical-ai-book plan follows constitution)
  - ✅ tasks-template.md (already aligned - 002-physical-ai-book tasks follows constitution)
  - ✅ All command templates verified for consistency
- Follow-up TODOs: None - all templates already aligned via 002-physical-ai-book feature workflow
- Changes: Complete scope redefinition for Panaversity Physical AI & Humanoid Robotics Hackathon (Deadline: Nov 30, 2025). Project now builds comprehensive textbook covering ROS 2, Gazebo/Unity, NVIDIA Isaac, and Vision-Language-Action models with RAG chatbot, Better-auth, content personalization, and Urdu translation. Target: 300 points (Base 100 + Better-auth 50 + Agent Skills 50 + Personalization 50 + Urdu 50).
-->

# Physical AI & Humanoid Robotics Textbook - Hackathon Constitution

## Project Context

**Hackathon**: Panaversity Physical AI & Humanoid Robotics
**Deadline**: November 30, 2025 at 6:00 PM
**Target Score**: 300 points (Base 100 + All Bonuses)
**Course Duration**: 13 weeks (42 hours total instruction)
**Format**: Comprehensive textbook with integrated RAG chatbot

## Core Principles

### I. Spec-Driven Development (NON-NEGOTIABLE)
Every significant feature MUST follow the Spec-Kit Plus workflow: Constitution → Specify → Plan → Tasks → Implement. No feature implementation begins without a specification, plan, and task breakdown. This ensures clarity, traceability, and reusable intelligence capture.

**Rationale**: Spec-driven development transforms ad-hoc coding into systematic, documented, and reusable intelligence. Each feature generates both working code and permanent reasoning artifacts (specs, plans, ADRs, prompt histories) that compound value across projects.

### II. Reusable Intelligence as First-Class Artifacts
Code is ephemeral; intelligence is permanent. Every feature MUST capture reasoning patterns, architectural decisions, and effective prompts as first-class artifacts (ADRs, PHRs, specs, plans). These artifacts are read by AI agents and humans to accelerate future work.

**Hackathon Application**: Physical AI content creation patterns (ROS 2 explanations, Gazebo simulation examples, Isaac platform tutorials) MUST be captured as PHRs. Agent skills for robotics education (SummarizeROSConcepts, GenerateGazeboQuiz, ExplainVLAModels) MUST use P+Q+P pattern (Persona + Questions + Principles) for reusability.

**Rationale**: Intelligence compounds—Project 1 teaches lessons that Project 10 inherits. ADRs explain "WHY" behind decisions; PHRs log what prompts work vs fail for Physical AI content generation; specs and plans become reusable templates for robotics education features.

### III. AI-Driven Development with Clear Boundaries
Use AI tools (Claude Code, OpenAI Agents SDK, ChatKit SDK) to accelerate development, but maintain clear boundaries: AI generates code based on specifications; humans review, test, and own the final implementation. All AI interactions that produce effective results MUST be logged as prompt history records.

**Rationale**: AI tools are force multipliers when guided by clear specifications. Logging effective prompts creates a searchable playbook for future AI collaboration, avoiding repeated mistakes.

### IV. Educational Clarity and Simplicity for Physical AI Learners
All code MUST be simple, readable, and well-commented so students learning robotics, ROS 2, Gazebo, and NVIDIA Isaac can understand implementations. Avoid over-engineering. Prefer clear, straightforward implementations over clever abstractions unless complexity is justified and documented in ADRs.

**Target Audience**: Students with varying backgrounds (software: Python/C++/ROS experience from beginner to advanced; hardware: RTX GPU, Jetson Orin kits, robot hardware access). Content MUST adapt to user background through personalization features.

**Rationale**: This is a hackathon project for teaching Physical AI. Complex code that works but is incomprehensible defeats the educational purpose. Simple, documented code teaches ROS 2 nodes, Gazebo simulations, and Isaac pipelines better and is easier to maintain.

### V. Security and Secrets Management (NON-NEGOTIABLE)
All sensitive information (OpenAI API keys, Neon Postgres connection strings, Qdrant API keys, Better-auth secrets) MUST be kept out of source control. Secrets MUST be loaded only from environment variables or secure configuration files (e.g., `.env` files that are gitignored). Never hard-code credentials.

**Rationale**: Exposed credentials compromise security and violate best practices. Environment-based configuration is standard, testable, and secure.

### VI. Architecture Decision Documentation
All major architecture decisions (framework choice, RAG strategy, chunking strategy, Neon Postgres schema, Better-auth integration, personalization approach, Urdu translation implementation) MUST be documented as ADRs (Architectural Decision Records) using Spec-Kit Plus templates. ADRs explain the "WHY" behind choices, not just the "WHAT" was built.

**Rationale**: ADRs create institutional memory. Future developers (and AI agents) understand why FastAPI was chosen over Flask, why JSONB columns store user backgrounds, why OpenAI handles personalization vs custom models, etc. This prevents repeated debates and enables informed evolution.

## Technical Stack

### Required Technologies (Hackathon Specification)
- **Frontend/Book**: Docusaurus for the textbook site and documentation, deployed to GitHub Pages
- **Backend API**: Python FastAPI for the backend API
- **Database**: Neon Serverless Postgres for user profiles, conversation history, and translation cache
- **Vector Database**: Qdrant Cloud Free Tier for storing Physical AI textbook embeddings (ROS 2, Gazebo, Isaac, VLA content)
- **AI Services**:
  - OpenAI for embeddings (text-embedding-3-small) and chat completions (gpt-4o-mini or gpt-4)
  - **MUST use both OpenAI Agents SDK and OpenAI ChatKit SDK** as required by hackathon specifications
- **Authentication**: Better-auth for JWT-based signup/signin with user background questionnaire
- **Source Control**: GitHub for source control and CI/CD (GitHub Actions for deployment)
- **Configuration**: Environment variables or `.env` files for all API keys and secrets (never hard-coded)

### Stack Rationale
This stack balances educational value, free-tier availability, and production-readiness. Docusaurus provides excellent documentation tooling for Physical AI course structure; FastAPI is modern and Python-friendly for robotics developers; Neon Postgres enables conversation persistence without infrastructure management; Qdrant Cloud Free Tier enables vector search for RAG; OpenAI provides reliable embeddings and chat completions; Better-auth simplifies authentication with customizable user profiles.

## Quality Requirements

### Physical AI Textbook Standards

The textbook MUST cover the complete 13-week Physical AI & Humanoid Robotics course:

#### **Module 1: ROS 2 Fundamentals (Weeks 3-5, 9 hours)**
- **Week 3-4**: ROS 2 Architecture
  - ROS 2 nodes, topics, services, and actions
  - Python client library (rclpy) for robot control
  - URDF (Unified Robot Description Format) for humanoid modeling
- **Week 5**: ROS 2 Advanced
  - Launch files, parameter servers, custom message types
  - ROS 2 package for simple humanoid control

#### **Module 2: Gazebo & Unity Simulation (Weeks 6-7, 6 hours)**
- **Week 6**: Gazebo Simulation
  - Physics simulation fundamentals
  - URDF/SDF (Simulation Description Format) integration
  - Sensor simulation (LiDAR, Depth Cameras, IMUs)
- **Week 7**: Unity for Robotics
  - Unity ML-Agents integration
  - Comparative analysis: Gazebo vs Unity for humanoid sim

#### **Module 3: NVIDIA Isaac Platform (Weeks 8-10, 9 hours)**
- **Week 8-9**: Isaac Sim
  - Photorealistic simulation (RTX rendering)
  - Synthetic data generation for training
  - Isaac ROS integration (ROS 2 packages with hardware acceleration)
- **Week 10**: Isaac for Navigation
  - Isaac ROS VSLAM (Visual SLAM) for localization
  - Nav2 path planning adapted for bipedal locomotion

#### **Module 4: Vision-Language-Action Models (Week 13, 3 hours)**
- Convergence of LLMs and robotics
- OpenAI Whisper for voice-to-action commands
- LLM-based cognitive planning for humanoid tasks
- Capstone project: Conversational humanoid robot

#### **Supporting Content (Weeks 1-2, 11-12)**
- **Weeks 1-2**: Introduction to Physical AI & Embodied Intelligence (6 hours)
  - Course overview, learning objectives, assessment structure
  - Introduction to embodied AI and physical intelligence concepts
  - Historical context: From industrial automation to humanoid robotics
- **Hardware Requirements Section**:
  - Minimum: RTX 4070 Ti or RTX 4080 (12-16GB VRAM), Ubuntu 22.04, 64GB RAM
  - Recommended: NVIDIA Jetson Orin Nano/NX/AGX, Intel RealSense Depth Camera D435/D455
  - Optional: Unitree Go2/G1 quadruped/humanoid robot hardware
- **Weekly Breakdown Table**: 13-week schedule with topics, hours, and deliverables
- **Weeks 11-12**: Humanoid Robot Development (6 hours) - Walking gaits, balance control, manipulation

#### **Assessments**
- Week 5: ROS 2 package submission (simple humanoid control)
- Week 7: Gazebo simulation project (sensor integration)
- Week 10: Isaac VSLAM navigation pipeline
- Week 13: Capstone conversational humanoid robot demo

#### **Textbook Quality Requirements**
- The site MUST build and run locally with a simple, documented command sequence (`npm install && npm start`)
- The site MUST deploy successfully to GitHub Pages via GitHub Actions
- All modules/chapters MUST be internally linked and navigable via Docusaurus sidebar
- Content MUST be clear, consistent, and educational (800-1,200 words per chapter)
- Code examples MUST be tested and runnable (ROS 2 Python nodes, Gazebo SDF files, Isaac scripts)
- All technical terms MUST be defined on first use (e.g., "URDF: Unified Robot Description Format")

### RAG Backend Standards (100 Points Base)
- The RAG backend MUST ingest all Physical AI textbook content into Qdrant as chunks with embeddings
  - Chunking strategy: 200-300 tokens per chunk with 50-token overlap
  - Embeddings: OpenAI text-embedding-3-small (1536 dimensions)
  - Metadata: module, chapter, section, chunk_id for citations
- The RAG backend MUST answer questions using ONLY the ingested textbook content as context
  - Use OpenAI ChatKit SDK for chat completions with retrieved context
  - Use OpenAI Agents SDK for conversational flow management
- The RAG backend MUST handle errors from Qdrant, Neon Postgres, and OpenAI gracefully with clear error messages
- The RAG backend MUST return answers with source citations (module, chapter, chunk IDs, relevance scores)
- If the answer cannot be found in the textbook context, the chatbot MUST clearly state: "I couldn't find information about [topic] in the Physical AI textbook. Please try rephrasing or ask about ROS 2, Gazebo, Isaac, or VLA models."
- The RAG backend MUST persist all conversations to Neon Postgres (user_id, question, answer, citations, timestamp)

### Selected-Text Mode Standards (Base Functionality)
- Selected-text mode MUST accept user-selected text (from any textbook page) and a question
- Selected-text mode MUST answer strictly based on that provided text, without pulling in unrelated external knowledge
- Selected-text mode MUST clearly tell the user if the answer cannot be found in that selected context
- Selected-text mode MUST NOT perform RAG queries when in selected-text mode—it uses only the provided context as prompt to OpenAI

### Neon Serverless Postgres Standards (Base + Bonus Features)
- Database MUST use Neon Serverless Postgres Free Tier (3 GiB storage, branching support)
- Database MUST include these tables:
  - **users**: id (SERIAL PRIMARY KEY), email (UNIQUE), password_hash, software_background (JSONB), hardware_background (JSONB), created_at, last_login
  - **conversations**: id (SERIAL PRIMARY KEY), user_id (FK), session_id, question, answer, citations (JSONB), question_type (rag/selected_text), timestamp
  - **translations** (for Urdu caching): id (SERIAL PRIMARY KEY), original_text_hash, original_text, translated_text, source_language, target_language, cached_at
- Connection string MUST be loaded from environment variable `NEON_DATABASE_URL` (never hard-coded)
- All queries MUST use parameterized statements to prevent SQL injection
- Database migrations MUST be managed via Alembic or raw SQL scripts in `backend/migrations/`

### Better-auth Integration Standards (+50 Points)
- Better-auth MUST implement JWT-based authentication with signup and signin endpoints
- Signup MUST include user background questionnaire:
  - **Software Background** (JSONB): programming_languages (array), robotics_experience (none/beginner/intermediate/advanced), ai_ml_level (none/beginner/intermediate/advanced)
  - **Hardware Background** (JSONB): rtx_gpu_access (boolean), rtx_gpu_model (string or null), jetson_kit (boolean), robot_hardware (array: none/quadruped/humanoid/manipulator)
- JWT tokens MUST be stored securely (httpOnly cookies for production, localStorage acceptable for development)
- Protected routes MUST validate JWT tokens and return 401 Unauthorized for invalid/expired tokens
- User profile endpoint (`/auth/me`) MUST return user email and backgrounds for personalization

### Content Personalization Standards (+50 Points)
- Personalization MUST adjust textbook content complexity based on user background:
  - **Beginner** (no robotics experience, no AI/ML): Simplified explanations, more code comments, step-by-step tutorials
  - **Intermediate** (some robotics/AI experience): Standard textbook content with moderate depth
  - **Advanced** (extensive robotics/AI experience, RTX GPU, Jetson, robot hardware): Technical depth, advanced optimizations, research paper references
- Personalization MUST use OpenAI API (gpt-4o-mini or gpt-4) to rewrite content based on user profile
- Personalization MUST provide "Personalize for Me" button on each textbook page
- Personalized content MUST include indicator: "✨ Personalized for your [beginner/intermediate/advanced] level"
- Personalization MUST NOT cache results—each request generates fresh content based on current user background

### Urdu Translation Standards (+50 Points)
- Translation MUST convert English textbook content to Urdu using OpenAI API
- Translation MUST support RTL (right-to-left) text rendering with proper CSS (`.rtl-text { direction: rtl; text-align: right; }`)
- Translation MUST cache results in Neon Postgres `translations` table to reduce API costs
- Translation MUST provide "Translate to Urdu" button on each textbook page
- Translation MUST use Noto Nastaliq Urdu font for proper Urdu rendering
- Translation MUST maintain code blocks in English (do not translate Python/ROS 2 code)
- Translation MUST handle technical terminology consistently (e.g., "ROS 2" → "ROS 2", "URDF" → "URDF")
- Docusaurus i18n MUST be configured with Urdu locale (`ur`) for language switching

### Reusable Agent Skills Standards (+50 Points)
- Agent skills MUST follow P+Q+P pattern (Persona + Questions + Principles) for reusability
- Agent skills MUST be documented in `.claude/skills/` or similar directory structure
- Required skills (minimum 3 for full points):
  1. **SummarizeROSConcepts**: Summarize ROS 2 topics (nodes, topics, services, URDF) in 3-5 bullet points
  2. **GenerateGazeboQuiz**: Generate 5 multiple-choice questions from Gazebo simulation chapters
  3. **ExplainVLATerms**: Explain Vision-Language-Action model terminology in simple language
- Skills MUST expose clear API endpoints (e.g., `/skills/summarize`, `/skills/quiz`, `/skills/explain`)
- Skills MUST return structured JSON responses with metadata (skill_name, input, output, timestamp)
- Skills MUST be reusable across other robotics education projects (documented for future use)

### Code Quality Standards (All Features)
- Code MUST be simple, readable, and well-commented for Physical AI students learning robotics
- Code MUST avoid unnecessary over-engineering (follow YAGNI: You Aren't Gonna Need It)
- All major functions and classes MUST have docstrings (Google style or NumPy style)
- Error handling MUST be explicit and user-friendly:
  - Qdrant errors: "Vector database temporarily unavailable. Please try again."
  - OpenAI errors: "AI service error. Please check your API key configuration."
  - Neon Postgres errors: "Database connection failed. Please contact support."
- Logging MUST be included for debugging and monitoring (use Python `logging` module, INFO level minimum)

## Spec-Driven Development and Reusable Intelligence

### Feature Specification Requirements
Every significant feature (Physical AI textbook modules, RAG backend, chatbot UI, selected-text mode, Better-auth, personalization, Urdu translation, agent skills) MUST have:
- A specification in `.specify/specs/002-physical-ai-book/spec.md` describing what it does, inputs/outputs, and acceptance criteria
- An implementation plan in `.specify/specs/002-physical-ai-book/plan.md` describing architecture and main steps
- A task list in `.specify/specs/002-physical-ai-book/tasks.md` breaking work into small, testable steps with dependencies

### Prompt History Capture
When particularly effective prompts are discovered (for generating Physical AI content, ROS 2 tutorials, Gazebo examples, Isaac platform guides, RAG prompts, or agent skills), they MUST be captured as prompt history records (PHRs) in `history/prompts/002-physical-ai-book/` for reuse. PHRs document what prompts work vs fail, creating a searchable playbook for future AI collaboration on robotics education projects.

**Physical AI Content Patterns to Capture**:
- Effective prompts for explaining ROS 2 concepts (nodes, topics, services, URDF)
- Successful Gazebo simulation tutorial structures (physics, sensors, URDF/SDF integration)
- Proven Isaac platform explanation patterns (Isaac Sim, Isaac ROS, VSLAM, Nav2)
- Working Vision-Language-Action model tutorial formats (Whisper, LLM planning, capstone projects)

### Reusable Agent Skills Design (+50 Points Bonus)
Design reusable intelligence using Claude Code Subagents and Agent Skills. Create reusable skills/subagents (SummarizeROSConcepts, GenerateGazeboQuiz, ExplainVLATerms) using clear persona, questions, and principles (P+Q+P pattern) so they can be reused in future robotics education projects. Skills MUST be documented with their persona, analytical questions, and decision principles. This component qualifies for extra marks in hackathon evaluation.

## Governance

This Constitution supersedes all other development practices for this project. All PRs and reviews MUST verify compliance with Constitution principles.

### Amendment Procedure
- Amendments require documentation of rationale and impact
- Constitution version MUST follow semantic versioning:
  - **MAJOR**: Backward incompatible governance/principle removals or redefinitions (e.g., 1.0.1 → 2.0.0 for complete scope change)
  - **MINOR**: New principle/section added or materially expanded guidance
  - **PATCH**: Clarifications, wording, typo fixes, non-semantic refinements
- All amendments MUST update the Sync Impact Report at the top of this file
- Amendments MUST propagate to dependent templates (spec, plan, tasks) for consistency

### Compliance Review
- **Before implementation**: Verify specification aligns with Constitution principles and hackathon requirements
- **During planning**: Verify plan respects Constitution constraints (security, simplicity, spec-driven flow, all 300 points features included)
- **During implementation**: Verify code follows Constitution standards (security, clarity, documentation, Physical AI educational value)
- **Before merge**: Verify all Constitution requirements are met (ADRs for major decisions, PHRs for effective prompts, specs/plans/tasks for all features)

### Hackathon Compliance Checklist
Before final submission (Nov 30, 2025 at 6:00 PM), verify:
- ✅ Base functionality (100 points): Docusaurus textbook + RAG chatbot + Neon Postgres + Qdrant
- ✅ Better-auth (+50 points): Signup/signin with user background questionnaire
- ✅ Reusable agent skills (+50 points): Minimum 3 skills with P+Q+P pattern documented
- ✅ Content personalization (+50 points): OpenAI-based complexity adjustment with "Personalize for Me" button
- ✅ Urdu translation (+50 points): OpenAI translation with RTL rendering and caching
- ✅ Physical AI textbook content: All 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA) with intro, hardware requirements, weekly breakdown
- ✅ OpenAI Agents SDK and ChatKit SDK both integrated (hackathon requirement)
- ✅ GitHub Pages deployment successful with custom domain (if applicable)
- ✅ 90-second demo video created and submitted
- ✅ All secrets in environment variables (never hard-coded)
- ✅ README with setup instructions and demo video link

### Complexity Justification
If any feature requires violating Constitution principles (e.g., adding unnecessary complexity), it MUST be:
1. Documented in an ADR explaining why the violation is necessary
2. Justified with specific problem statements and rejected simpler alternatives
3. Reviewed and approved before implementation

**Version**: 2.0.0 | **Ratified**: 2025-01-28 | **Last Amended**: 2025-01-28
