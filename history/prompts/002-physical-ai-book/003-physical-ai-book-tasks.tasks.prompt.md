---
id: phr-20251127-003-tasks
title: Physical AI Textbook Task Breakdown
stage: tasks
date: 2025-11-27
surface: agent
model: claude-sonnet-4-5-20250929
feature: 002-physical-ai-book
branch: 002-physical-ai-book
user: talal
command: /sp.tasks
labels: [tasks, hackathon, physical-ai, task-breakdown, user-stories]
links:
  spec: specs/002-physical-ai-book/spec.md
  plan: specs/002-physical-ai-book/plan.md
  tasks: specs/002-physical-ai-book/tasks.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-physical-ai-book/tasks.md
tests:
  - N/A (task planning phase)
---

## Prompt

User command: `/sp.tasks` (no arguments)

**Context**: After creating comprehensive specification and implementation plan for Physical AI textbook hackathon project, proceed to generate dependency-ordered task breakdown organized by user story priorities.

**User Instruction**: "follow the docs i shared at all cost during the development at all cost" (strict compliance with hackathon documentation requirements)

## Response snapshot

Created comprehensive task breakdown for Physical AI & Humanoid Robotics textbook project (002-physical-ai-book):

**Task Organization**: 120 tasks across 8 phases, organized by user story to enable independent implementation and testing

**Phase Structure**:
1. **Phase 1: Setup** (14 tasks) - Project initialization, external services (Neon, Qdrant, OpenAI), environment configuration
2. **Phase 2: Foundational** (16 tasks) - Shared infrastructure (database layer, auth services, RAG pipeline reuse from 001-hackathon-app)
3. **Phase 3: US1 - Access Educational Textbook (P0-MVP)** (32 tasks) - Write 20+ chapters covering Physical AI course content (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA, hardware requirements)
4. **Phase 4: US2 - RAG with Conversation Persistence (P0-MVP)** (9 tasks) - Integrate RAG chatbot with Neon Postgres for conversation history
5. **Phase 5: US3 - Ask Questions About Selected Text (P1)** (6 tasks) - Selected-text mode without RAG queries
6. **Phase 6: US4 - Sign Up and Provide Background (P1-Bonus +50)** (16 tasks) - Better-auth with user background questionnaire
7. **Phase 7: US5 - Use Reusable Agent Skills (P2-Bonus +50)** (6 tasks) - Verify agent skills work with Physical AI content
8. **Phase 8: Polish & Cross-Cutting Concerns** (21 tasks) - ADRs, deployment, demo video, hackathon submission

**Key Task Characteristics**:

1. **User Story Organization**: Tasks grouped by user story (US1-US5) to enable independent testing and incremental delivery
2. **Strict Format Compliance**: All 120 tasks follow checklist format `- [ ] [TaskID] [P?] [Story?] Description with file path`
3. **Parallelization Marked**: 47 tasks marked with [P] can run in parallel (different files, no blocking dependencies)
4. **Hackathon Documentation Adherence**: All textbook content tasks (T037-T060) explicitly reference "per hackathon documentation" to ensure strict compliance with course requirements
5. **Infrastructure Reuse**: Phase 2 includes 10 tasks copying code from 001-hackathon-app (marked as REUSE), maximizing efficiency

**Task Breakdown by User Story**:
- **US1 (Textbook Content)**: 32 tasks (T031-T062) - Longest phase at 16-20 hours estimated effort
  - Docusaurus configuration: 6 tasks
  - Introduction & Hardware content: 7 tasks
  - Module 1 (ROS 2): 4 chapters
  - Module 2 (Gazebo & Unity): 4 chapters
  - Module 3 (NVIDIA Isaac): 4 chapters
  - Module 4 (VLA): 3 chapters + capstone
  - Review & testing: 2 tasks

- **US2 (RAG + Neon Postgres)**: 9 tasks (T063-T071) - Backend extension + frontend chatbot integration

- **US3 (Selected Text)**: 6 tasks (T072-T077) - Selected-text endpoint + frontend handler

- **US4 (Better-auth)**: 16 tasks (T078-T093) - Backend auth endpoints (3) + frontend UI (6) + integration (2) + testing (2)

- **US5 (Agent Skills)**: 6 tasks (T094-T099) - Verification only (skills already implemented from 001-hackathon-app)

**Parallel Execution Opportunities**:
- Phase 1: 5 parallel tasks (account creation, requirements updates)
- Phase 2: 11 parallel tasks (copying from 001-hackathon-app, new models)
- Phase 3: 24 parallel tasks (content writing for different chapters/modules)
- Phase 6: 9 parallel tasks (backend endpoints, frontend UI components)
- Phase 8: 7 parallel tasks (ADRs, deployment preparation)

**Critical Path** (longest dependency chain):
Setup → Foundational → US1 Content Writing (32 tasks, ~16-20 hours) → US2 Integration → US4 Better-auth → Final Testing & Deployment

**Hackathon Documentation Compliance**:

All textbook content tasks ensure strict adherence to hackathon requirements:
- **ROS 2 (Module 1)**: Architecture, nodes/topics/services/actions, rclpy Python integration, URDF for humanoids
- **Gazebo & Unity (Module 2)**: Physics simulation, URDF/SDF formats, sensor simulation (LiDAR, Depth Cameras, IMUs)
- **NVIDIA Isaac (Module 3)**: Isaac Sim (photorealistic simulation), Isaac ROS (VSLAM, hardware acceleration), Nav2 path planning for bipedal movement
- **VLA (Module 4)**: OpenAI Whisper voice-to-action, LLM cognitive planning, capstone project ("robot receives voice command, plans path, navigates obstacles, identifies object, manipulates it")
- **Hardware Requirements**: RTX 4070 Ti+, Ubuntu 22.04, 64GB RAM, Jetson Orin kits (Nano 8GB, NX 16GB), RealSense cameras, Unitree Go2/G1 robots, cloud alternatives
- **13-Week Course Structure**: Weeks 1-2 (Intro), 3-5 (ROS 2), 6-7 (Gazebo/Unity), 8-10 (Isaac), 11-12 (Humanoid Dev), 13 (Conversational Robotics)
- **Assessments**: ROS 2 package project, Gazebo simulation, Isaac perception pipeline, autonomous humanoid capstone

**Execution Strategy**:
1. Complete Setup and Foundational phases sequentially (blocking prerequisites)
2. Start US1 content writing immediately after Phase 2 (longest phase, use AI assistance for drafts)
3. Parallelize US2 RAG integration while US1 content writing in progress
4. Add US3 and US4 incrementally after US2 complete
5. US5 is quick verification (minimal effort, skills already done)
6. Phase 8 polish and deployment requires all user stories complete

**MVP Scope** (100 base points):
- Phase 1: Setup
- Phase 2: Foundational
- Phase 3: US1 (can shorten to ~10 core chapters if time-constrained)
- Phase 4: US2 (RAG + Neon Postgres)
- Phase 5: US3 (Selected Text)
- Phase 8: Deployment & Demo Video

**Bonus Scope** (200 points target):
- Add Phase 6: US4 (Better-auth) for +50 points
- Add Phase 7: US5 (Agent Skills) for +50 points (already done, just verify)

**Estimated Total Effort**: 48-60 hours for solo developer (matches plan.md timeline)

**Independent Test Criteria**:
- **US1**: Deploy textbook to GitHub Pages, verify 20+ chapters accessible, navigation works, content matches hackathon requirements
- **US2**: Ask questions via chatbot, verify RAG answers with citations, check Neon Postgres conversations table for saved data
- **US3**: Select text, click "Ask about this selection," verify answer uses only selected text, check Neon Postgres for question_type='selected_text'
- **US4**: Sign up with background questionnaire, sign in, verify profile in Neon Postgres users table, check authenticated chatbot saves user_id
- **US5**: Test all 3 agent skills (summarize, quiz, explain) with Physical AI content, verify P+Q+P documentation

## Outcome

- ✅ Impact: Comprehensive task breakdown with 120 tasks organized by user story, enabling independent implementation and testing. Clear dependencies, parallelization opportunities, and strict hackathon documentation compliance.
- 🧪 Tests: Independent test criteria defined for each user story phase
- 📁 Files: Created tasks.md with full task breakdown
- 🔁 Next prompts: Begin implementation starting with Phase 1 (Setup), then Phase 2 (Foundational), then Phase 3 (US1 content writing - use AI assistance for chapter drafts)
- 🧠 Reflection: Task organization by user story enables incremental delivery and independent testing. Phase 3 (US1) is critical path at 32 tasks (~16-20 hours) - maximize parallelization and AI assistance for content drafts. Strict format compliance (120 tasks all follow checkbox + ID + [P]/[Story] + description + file path). All textbook content tasks explicitly reference hackathon documentation to ensure compliance. Infrastructure reuse (60% from 001-hackathon-app) accelerates development. Target 200 points achievable in 48-60 hours with disciplined execution.

## Evaluation notes (flywheel)

- Failure modes observed: None (task breakdown completed successfully)
- Graders run and results (PASS/FAIL): Task format validation PASS (all 120 tasks follow strict checklist format)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementation with Phase 1 Setup tasks (T001-T014), particularly focusing on external service setup (Neon Postgres, Qdrant, OpenAI) and database schema deployment. Then proceed to Phase 2 Foundational to set up shared infrastructure before tackling Phase 3 content creation (longest phase).
