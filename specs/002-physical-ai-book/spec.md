# Feature Specification: Physical AI & Humanoid Robotics Textbook

**Feature Branch**: `002-physical-ai-book`
**Created**: 2025-11-27
**Status**: Draft
**Hackathon**: Panaversity Physical AI & Humanoid Robotics Course Textbook
**Deadline**: Sunday, Nov 30, 2025 at 06:00 PM
**Hackathon Target**: 300 points (Base 100 + Better-auth 50 + Personalization 50 + Urdu 50 + Agent Skills 50)

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Access Educational Textbook on Physical AI (Priority: P0 - MVP)

A student enrolled in the Physical AI & Humanoid Robotics course wants to learn about embodied intelligence, ROS 2, Gazebo simulation, NVIDIA Isaac platform, and humanoid robot development. They access the textbook through a web browser, navigate through weekly modules, and study comprehensive content covering all 13 weeks of the course.

**Why this priority**: This is the foundational deliverable for the hackathon. The textbook must cover all course modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA) with educational content suitable for students learning Physical AI. Without this content, the project fails to meet the hackathon's core requirement.

**Independent Test**: Can be fully tested by deploying the book to GitHub Pages, accessing it via URL, navigating through all modules (Introduction, ROS 2, Gazebo/Unity, NVIDIA Isaac, Humanoid Development, Conversational Robotics), and verifying content matches course learning outcomes.

**Acceptance Scenarios**:

1. **Given** the textbook is deployed to GitHub Pages, **When** a student visits the textbook URL, **Then** they see a homepage introducing Physical AI with clear navigation to all modules
2. **Given** a student is studying Module 1 (ROS 2), **When** they navigate through the content, **Then** they can access chapters on ROS 2 Nodes, Topics, Services, Python integration (rclpy), and URDF for humanoids
3. **Given** a student wants to learn about simulation, **When** they access Module 2, **Then** they find comprehensive content on Gazebo physics simulation, Unity rendering, and sensor simulation (LiDAR, Depth Cameras, IMUs)
4. **Given** a student is studying NVIDIA Isaac, **When** they access Module 3, **Then** they find chapters on Isaac Sim, Isaac ROS, VSLAM, and Nav2 path planning
5. **Given** a student wants to understand VLA (Vision-Language-Action), **When** they access Module 4, **Then** they find content on OpenAI Whisper for voice commands, LLM-based cognitive planning, and the autonomous humanoid capstone project
6. **Given** a student needs hardware guidance, **When** they access the Hardware Requirements section, **Then** they find detailed information about workstation requirements (RTX GPUs, Ubuntu 22.04), Jetson Orin kits, RealSense cameras, and robot options (Unitree Go2/G1)

---

### User Story 2 - Ask Questions Using RAG with Conversation Persistence (Priority: P0 - MVP)

A student wants to ask questions about textbook content and receive accurate answers based on the book's information. They interact with an embedded RAG chatbot that queries relevant content, generates answers using OpenAI Agents/ChatKit SDK, and persists conversation history in Neon Serverless Postgres for future reference.

**Why this priority**: This is a core hackathon requirement demonstrating RAG capabilities with proper database integration (Neon Postgres). It provides interactive learning and satisfies the technical requirement for conversation storage.

**Independent Test**: Can be tested by asking questions about Physical AI concepts, verifying answers come from textbook context, checking source citations, and confirming conversations are saved to Neon Postgres database.

**Acceptance Scenarios**:

1. **Given** textbook content is ingested into Qdrant, **When** a student asks "What is Physical AI?", **Then** they receive an answer from the introduction chapter with source citations
2. **Given** a student asks "How do ROS 2 nodes communicate?", **When** the RAG system processes the query, **Then** it retrieves relevant chunks from Module 1, uses OpenAI ChatKit SDK to generate an answer, and displays it with chapter references
3. **Given** a student asks a question, **When** the answer is generated, **Then** the conversation (question, answer, timestamp, user context) is stored in Neon Serverless Postgres for history tracking
4. **Given** a student asks about NVIDIA Isaac Sim, **When** the system cannot find relevant content, **Then** it clearly states the answer cannot be found in the textbook instead of hallucinating
5. **Given** backend errors occur (Qdrant unavailable, OpenAI rate limit, Postgres connection failure), **When** the student submits a question, **Then** they see clear, user-friendly error messages without technical implementation details

---

### User Story 3 - Ask Questions About Selected Text (Priority: P1)

A student wants to ask focused questions about a specific code example, hardware specification, or technical concept they've highlighted in the textbook. They select text, click a button to ask about it, and receive answers based only on that selected context without RAG queries.

**Why this priority**: This is a required hackathon deliverable. It enables students to ask targeted questions about specific sections (e.g., "Explain this URDF code snippet") without retrieving unrelated content.

**Independent Test**: Can be tested by selecting text from any module, clicking "Ask about this selection," asking a question, and verifying the answer uses only the selected text as context.

**Acceptance Scenarios**:

1. **Given** a student is reading about URDF format, **When** they select a code example and highlight it, **Then** a button appears offering "Ask about this selection"
2. **Given** a student selects text about Jetson Orin specifications and clicks the ask button, **When** they type "What is the VRAM of this device?", **Then** the chatbot answers using only the selected text without querying the vector database
3. **Given** selected text about Nav2 path planning is provided, **When** the student asks a question that cannot be answered from that text, **Then** the system clearly states the answer cannot be found in the selected context
4. **Given** a student selects an empty string or very short text (< 10 characters), **When** they attempt to ask about it, **Then** the system validates and rejects with a clear message requiring minimum text length

---

### User Story 4 - Sign Up and Provide Background Information (Priority: P1 - Bonus)

A new student wants to create an account to personalize their learning experience. They sign up using Better-auth, provide information about their software background (Python, C++, robotics experience) and hardware background (access to RTX GPU, Jetson kit, robot hardware), which is stored in Neon Postgres for future personalization.

**Why this priority**: This bonus feature (+50 points) enables user identification and lays the foundation for personalized content. Collecting background information helps tailor the learning experience to students with different skill levels and hardware access.

**Independent Test**: Can be tested by signing up with email/password, filling out the background questionnaire, logging in, and verifying user profile is stored in Neon Postgres with background data.

**Acceptance Scenarios**:

1. **Given** a new student visits the textbook, **When** they click "Sign Up," **Then** they see a registration form powered by Better-auth with email and password fields
2. **Given** a student completes registration, **When** they submit the form, **Then** they are asked to answer questions about their software background (programming languages, prior robotics experience, AI/ML knowledge level)
3. **Given** a student is answering background questions, **When** they complete the software background section, **Then** they are asked about hardware availability (RTX GPU access, Jetson Orin kit ownership, robot hardware access)
4. **Given** a student completes the background questionnaire, **When** they finish, **Then** their profile (email, hashed password, software background, hardware background) is stored in Neon Postgres and they are logged in
5. **Given** a returning student visits the textbook, **When** they click "Sign In" and provide credentials, **Then** Better-auth authenticates them and they access the textbook with their stored profile

---

### User Story 5 - Use Reusable Agent Skills for Learning (Priority: P2 - Bonus)

A student wants to use specialized learning tools to summarize complex sections, generate quiz questions for self-assessment, or get detailed explanations of robotics terms. They invoke reusable agent skills (SummarizeSection, GenerateQuizQuestions, ExplainTerm) implemented using Claude Code Subagents pattern with Persona + Questions + Principles.

**Why this priority**: This bonus feature (+50 points) demonstrates reusable intelligence and advanced agent design. Skills enhance the learning experience by providing study aids. Since these are already implemented from the previous project, they can be reused with minimal adaptation.

**Independent Test**: Can be tested by invoking each skill with textbook content, verifying outputs match educational purposes, and confirming skills follow P+Q+P pattern documentation.

**Acceptance Scenarios**:

1. **Given** a student wants to review Module 1, **When** they invoke SummarizeSection with a chapter on ROS 2 Nodes, **Then** they receive a concise summary preserving key concepts about nodes, topics, and services
2. **Given** a student wants to test their knowledge, **When** they invoke GenerateQuizQuestions with content about NVIDIA Isaac Sim, **Then** they receive multiple-choice questions with plausible distractors and explanations
3. **Given** a student encounters the term "VSLAM," **When** they invoke ExplainTerm, **Then** the system uses RAG to find relevant textbook context and explains Visual SLAM in the context of Isaac ROS
4. **Given** agent skills are implemented, **When** reviewed for reusability, **Then** each skill is documented with persona (e.g., "Educational Content Specialist"), analytical questions, and decision principles following the P+Q+P pattern

---

### Edge Cases

- What happens when a student has no hardware access (no RTX GPU, no Jetson)? Textbook should provide cloud-based alternatives (AWS RoboMaker, Omniverse Cloud) and simulation-only learning paths
- How does the system handle questions about topics not covered in the textbook (e.g., "How do I program Boston Dynamics Spot?")? System should clearly indicate content is limited to course scope and suggest related textbook sections
- What happens when selected text contains complex tables or diagrams? System should extract text content while acknowledging visual elements may not be fully represented
- How does the system handle concurrent user registrations with the same email? Better-auth should enforce unique email constraints and return clear error messages
- What happens when Neon Postgres connection fails during conversation storage? System should still deliver the answer to the user but log the storage failure and retry asynchronously
- How does the system handle students with partial hardware (e.g., RTX GPU but no Jetson)? Background questionnaire should capture granular hardware availability for future personalization
- What happens when a student asks a question requiring information from multiple modules (e.g., "How does Isaac Sim integrate with ROS 2?")? RAG should retrieve chunks from both modules and synthesize a comprehensive answer
- How does the system handle very long questions or selected text exceeding token limits? System should enforce reasonable length limits (e.g., 500 tokens for questions, 2000 tokens for selected text) with clear error messages

---

## Requirements *(mandatory)*

### Functional Requirements

#### Textbook Content (Base - 100 points)

- **FR-001**: System MUST provide a Docusaurus-based textbook covering Physical AI & Humanoid Robotics with modules for Introduction (Weeks 1-2), ROS 2 (Weeks 3-5), Gazebo/Unity Simulation (Weeks 6-7), NVIDIA Isaac (Weeks 8-10), Humanoid Development (Weeks 11-12), and Conversational Robotics (Week 13)
- **FR-001a**: Each textbook chapter MUST contain 800-1,200 words of educational content to ensure comprehensive coverage without overwhelming students
- **FR-002**: System MUST include an introduction module explaining Physical AI principles, embodied intelligence, the transition from digital AI to physical robots, and overview of the humanoid robotics landscape
- **FR-003**: System MUST include Module 1 content covering ROS 2 architecture, nodes/topics/services/actions, building ROS 2 packages with Python (rclpy), launch files, parameter management, and URDF for humanoid robots
- **FR-004**: System MUST include Module 2 content covering Gazebo simulation environment, URDF and SDF robot description formats, physics and sensor simulation, and Unity for high-fidelity rendering
- **FR-005**: System MUST include Module 3 content covering NVIDIA Isaac Sim (photorealistic simulation, synthetic data generation), Isaac ROS (hardware-accelerated VSLAM, navigation), and Nav2 path planning for bipedal locomotion
- **FR-006**: System MUST include Module 4 content covering Vision-Language-Action (VLA), voice-to-action with OpenAI Whisper, cognitive planning using LLMs to translate natural language to ROS 2 actions, and the autonomous humanoid capstone project
- **FR-007**: System MUST include a comprehensive hardware requirements section covering workstation specifications (RTX 4070 Ti+, Ubuntu 22.04, 64GB RAM), Jetson Orin kits, RealSense cameras, and robot options (Unitree Go2, G1, alternatives)
- **FR-008**: System MUST include weekly breakdown content mapping to the 13-week course structure with topics for each week
- **FR-009**: System MUST deploy the textbook to GitHub Pages and make it accessible via public URL
- **FR-010**: System MUST provide clear navigation and table of contents allowing students to access any module or chapter directly
- **FR-011**: System MUST support local development with documented build and run commands (npm install, npm start, npm run build)

#### RAG Backend with Database Integration (Base - 100 points)

- **FR-012**: System MUST use FastAPI as the backend framework for all API endpoints
- **FR-013**: System MUST ingest all textbook content into Qdrant Cloud Free Tier by chunking text, creating embeddings using OpenAI (text-embedding-3-small or similar), and storing chunks with metadata (module, chapter, page references)
- **FR-014**: System MUST provide POST /ingest endpoint that reads textbook markdown files, chunks content (200-300 tokens with 50-token overlap using tiktoken), creates embeddings, and stores in Qdrant
- **FR-015**: System MUST provide POST /ask endpoint that accepts a question, creates query embedding, retrieves top 5 relevant chunks from Qdrant, uses BOTH OpenAI Agents SDK (for conversational flow management) AND ChatKit SDK (for chat completions with retrieved context) to generate answers, and returns answer with source citations
- **FR-016**: System MUST integrate Neon Serverless Postgres database to store conversation history including question text, answer text, timestamp, user ID (if authenticated), source citations, and session metadata
- **FR-017**: System MUST only use textbook content as context for RAG answers, with no external knowledge included in the generation process
- **FR-018**: System MUST return answers with source citations including module name, chapter reference, and relevance score
- **FR-019**: System MUST clearly state when an answer cannot be found in textbook context instead of hallucinating or providing incorrect information
- **FR-020**: System MUST handle errors from Qdrant (connection failures, query timeouts), OpenAI (rate limits, API errors), and Neon Postgres (connection failures, write errors) gracefully with clear, user-friendly error messages. System MUST handle CORS errors with appropriate error messages. System MUST handle invalid JWT tokens with 401 Unauthorized responses. System MUST handle duplicate email signup attempts with 400 Bad Request and clear error message. System MUST handle invalid credentials during signin with 401 Unauthorized and clear error message
- **FR-021**: System MUST provide POST /ask_selected endpoint that accepts a question and selected text context, generates an answer using only the provided text without querying Qdrant, and returns the answer with a citation to the user's selection

#### Chatbot UI (Base - 100 points)

- **FR-022**: System MUST provide a React chatbot component embedded in the Docusaurus textbook pages
- **FR-023**: System MUST display a chat interface where students can type questions and see conversation history
- **FR-024**: System MUST call POST /ask endpoint when students submit questions via the chat interface
- **FR-025**: System MUST display answers in chat format with clear distinction between user questions and bot responses
- **FR-026**: System MUST show source citations (module, chapter, relevance score) alongside answers when available
- **FR-027**: System MUST be responsive and functional on devices with screen widths from 320px (mobile) to 1920px (desktop)
- **FR-028**: System MUST allow students to select/highlight text anywhere in textbook pages
- **FR-029**: System MUST display a "Ask about this selection" button when text is selected (minimum 10 characters)
- **FR-030**: System MUST open the chat interface with selected text as context when the ask button is clicked
- **FR-031**: System MUST call POST /ask_selected endpoint with the selected text and question when answering selected-text queries

#### Authentication and User Background (Bonus - +50 points)

- **FR-032**: System MUST implement signup and signin functionality using Better-auth (https://www.better-auth.com/)
- **FR-033**: System MUST provide a signup form collecting email and password from new students
- **FR-034**: System MUST present a background questionnaire during signup with questions about software background including: programming languages known (Python, C++, JavaScript, other), prior robotics experience (none, beginner, intermediate, advanced), AI/ML knowledge level (none, basic, intermediate, advanced)
- **FR-035**: System MUST present a background questionnaire during signup with questions about hardware availability including: RTX GPU access (yes/no, model), Jetson Orin kit ownership (yes/no, model), robot hardware access (none, quadruped, humanoid, robotic arm)
- **FR-036**: System MUST store user profiles in Neon Serverless Postgres including email, hashed password (via Better-auth), software background responses, hardware background responses, and account creation timestamp
- **FR-037**: System MUST provide a signin form where returning students can authenticate using email and password. System MUST validate JWT tokens for protected routes and return 401 Unauthorized for invalid or expired tokens. JWT tokens MUST be stored in httpOnly cookies for production (with localStorage fallback for development)
- **FR-037a**: System MUST provide GET /auth/me endpoint that returns the authenticated user's profile including email, software background, and hardware background when a valid JWT token is provided
- **FR-038**: System MUST associate conversation history with authenticated users by storing user ID with each question/answer pair in Neon Postgres
- **FR-039**: System MUST allow unauthenticated users to read the textbook and use the chatbot with conversation history stored anonymously (session ID only)

#### Reusable Agent Skills (Bonus - +50 points)

- **FR-040**: System MUST provide a SummarizeSection skill that accepts textbook content and returns a concise summary preserving key educational concepts. System MUST provide POST /skills/summarize endpoint that accepts content and returns structured JSON with summary text and metadata
- **FR-041**: System MUST provide a GenerateQuizQuestions skill that accepts textbook content and returns multiple-choice questions with options, correct answer, and explanations. Plausible distractors MUST be related to the topic but clearly incorrect, avoiding obvious wrong answers. System MUST provide POST /skills/quiz endpoint that accepts content and returns structured JSON with questions, options, correct answer, and explanations
- **FR-042**: System MUST provide an ExplainTerm skill that accepts a term, uses RAG to find relevant textbook context, and returns a clear explanation in the context of Physical AI and robotics. Clear explanation MUST include definition, context, and example usage. System MUST provide POST /skills/explain endpoint that accepts a term and returns structured JSON with explanation and source citations
- **FR-043**: System MUST design all agent skills using the Persona + Questions + Principles (P+Q+P) pattern
- **FR-044**: System MUST document each agent skill with its persona (role and expertise), analytical questions (guided reasoning), and decision principles (consistent behavior guidelines)
- **FR-045**: System MUST make agent skills reusable across different textbook topics and future projects by following documented patterns

#### Content Personalization (Bonus - +50 points)

- **FR-046**: System MUST provide POST /personalize endpoint (protected by JWT authentication) that accepts a chapter_id and user_id, analyzes the user's software and hardware background from Neon Postgres, determines complexity level (beginner/intermediate/advanced) based on robotics_experience and ai_ml_level, and returns personalized content
- **FR-047**: System MUST use OpenAI ChatCompletion API to rewrite chapter content adjusting for complexity level (beginner: simpler language, more explanations; advanced: concise, assume prior knowledge) and include hardware alternatives if user lacks RTX GPU/Jetson (suggest cloud options like AWS RoboMaker, Omniverse Cloud)
- **FR-048**: System MUST provide a "Personalize for Me" button at the start of each chapter (visible only to authenticated users) that triggers content personalization
- **FR-049**: System MUST display a visual indicator when content is personalized (e.g., badge or banner) and allow users to toggle between original and personalized content
- **FR-050**: System MUST generate fresh personalized content on each request (no caching) to ensure content reflects current user background and preferences

#### Urdu Translation (Bonus - +50 points)

- **FR-051**: System MUST provide POST /translate endpoint (protected by JWT authentication) that accepts a chapter_id and target_language="urdu", checks translation cache in Neon Postgres, and if not cached, uses OpenAI ChatCompletion API or Google Translate API to translate English content to Urdu while preserving technical terms (ROS 2, URDF, NVIDIA Isaac) in English
- **FR-052**: System MUST provide a "Translate to Urdu" button at the start of each chapter (visible only to authenticated users) that triggers translation
- **FR-053**: System MUST render Urdu content with RTL (right-to-left) text direction using CSS (direction: rtl, text-align: right) while keeping code blocks and diagrams in LTR (left-to-right)
- **FR-054**: System MUST use Noto Nastaliq Urdu font or similar for proper Urdu script rendering
- **FR-055**: System MUST cache translations in Neon Postgres translations table to avoid re-translating same content and reduce API costs

### Key Entities *(include if feature involves data)*

- **Module**: Represents a major section of the textbook. Key attributes: module number (1-4), module title (e.g., "ROS 2 Robotic Nervous System"), description, weeks covered, learning outcomes
- **Chapter**: Represents a specific topic within a module. Key attributes: chapter title, content (markdown), module reference, order, code examples, diagrams
- **Text Chunk**: Represents a piece of textbook content stored in Qdrant. Key attributes: chunk text, embedding vector (1536 dimensions for text-embedding-3-small), module reference, chapter reference, chunk ID, token count
- **Question**: Represents a student's query. Key attributes: question text, timestamp, user ID (if authenticated), session ID, selected text context (if selected-text mode), question type (RAG or selected-text)
- **Answer**: Represents the chatbot's response. Key attributes: answer text, source citations (module, chapter, chunk ID, relevance score), model used (gpt-3.5-turbo or gpt-4), timestamp, question reference
- **Conversation**: Represents a stored conversation in Neon Postgres. Key attributes: conversation ID, user ID (if authenticated), session ID, question, answer, source citations JSON, timestamp, question type
- **User**: Represents a student with an account. Key attributes: user ID, email, hashed password (Better-auth), software background (programming languages, robotics experience, AI/ML level), hardware background (RTX GPU, Jetson kit, robot access), account created timestamp, last login timestamp
- **Agent Skill**: Represents a reusable intelligence capability. Key attributes: skill name (SummarizeSection, GenerateQuizQuestions, ExplainTerm), persona description, analytical questions list, decision principles list, input contract, output contract

---

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can access the textbook via GitHub Pages URL and navigate to any module within 2 clicks from the homepage
- **SC-002**: The textbook covers all 13 weeks of course content with at least 20 comprehensive chapters across 4 modules plus introduction and hardware sections
- **SC-003**: The textbook includes code examples for ROS 2 concepts, URDF definitions, and Python (rclpy) integration with proper syntax highlighting
- **SC-004**: The textbook builds successfully from source using "npm run build" in under 3 minutes on a standard development machine
- **SC-005**: Students can ask questions about textbook content and receive answers within 7 seconds (p95 latency), with 90% of answers accurately reflecting textbook content based on manual evaluation
- **SC-006**: The RAG system retrieves relevant chunks for 85% of questions, with retrieved chunks containing information sufficient to answer the question based on human review
- **SC-007**: All conversations (question, answer, timestamp, citations) are successfully stored in Neon Serverless Postgres within 2 seconds of answer generation, with 99% write success rate
- **SC-008**: Students can select text and ask questions about it, receiving answers based only on selected context in 95% of cases where selected text contains relevant information
- **SC-009**: The system clearly indicates when answers cannot be found (either in textbook or selected text) in 100% of cases, with zero hallucinated answers in testing
- **SC-010**: The chatbot interface is fully functional and usable on screen widths from 320px to 1920px, maintaining all features without horizontal scrolling or broken layouts
- **SC-011**: Students can sign up, complete the background questionnaire, and sign in using Better-auth with 100% success rate for valid credentials, with profiles stored in Neon Postgres
- **SC-012**: The background questionnaire captures software background (3+ questions) and hardware background (3+ questions) with responses stored in structured format in Neon Postgres
- **SC-013**: Agent skills (SummarizeSection, GenerateQuizQuestions, ExplainTerm) produce educationally valuable outputs in 90% of test cases with valid textbook content inputs. "Educationally valuable" means outputs are accurate, relevant, appropriate for educational context, and aid learning
- **SC-014**: All agent skills are documented with persona, analytical questions, and decision principles, enabling reuse in future educational projects
- **SC-015**: The system handles errors gracefully, displaying user-friendly messages in 100% of error scenarios (Qdrant unavailable, OpenAI rate limits, Postgres failures) without exposing technical stack traces

---

## Assumptions

- Students have access to modern web browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+) with JavaScript enabled (required for React chatbot and interactive features)
- The textbook content is written in English as the primary language (Urdu translation is a bonus feature)
- OpenAI API keys with sufficient quota for embeddings (text-embedding-3-small) and chat completions (gpt-3.5-turbo or gpt-4) are available and accessible
- Network connectivity is available for API calls to OpenAI, Qdrant Cloud, and Neon Postgres with average latency < 200ms
- Qdrant Cloud Free Tier account with API credentials is available and has sufficient storage for textbook content (estimated 500-1000 chunks)
- Neon Serverless Postgres account with connection string is available and has sufficient storage for user profiles and conversation history (estimated 1000+ conversations)
- Better-auth can be integrated with the Docusaurus/React frontend and FastAPI backend via JWT tokens or session cookies
- The FastAPI backend can be deployed to a cloud service (Render, Railway, Fly.io, or Vercel serverless functions) accessible by the GitHub Pages frontend via CORS
- Students have basic familiarity with web interfaces and can navigate documentation sites with table of contents
- Network connectivity is available for API calls to OpenAI, Qdrant Cloud, and Neon Postgres (average latency < 200ms)
- The textbook content is primarily text-based with code examples and diagrams (minimal video or interactive 3D content)
- Students understand that the textbook is for educational purposes and hardware requirements are informational (not all students will have access to RTX GPUs, Jetson kits, or robot hardware)
- The hackathon submission includes a public GitHub repository with source code and a demo video under 90 seconds
- The textbook will be evaluated by judges who understand Physical AI, ROS 2, and humanoid robotics concepts

---

## Out of Scope (for this hackathon)

- **Interactive 3D simulations**: Embedding actual Gazebo or Isaac Sim simulations in the browser is out of scope; the textbook provides conceptual education and screenshots/diagrams only
- **Real-time collaboration**: Students cannot collaborate in real-time on questions or share conversation history with peers
- **Video content**: The textbook does not include video tutorials or recorded lectures; content is text, images, and code examples only
- **Mobile app**: The textbook is web-based only; native iOS/Android apps are not required
- **Payment or subscription**: All content is free and accessible without payment
- **Admin dashboard**: No admin interface for managing users, content, or analytics is required for this hackathon
- **Content versioning**: The textbook content is considered static for the hackathon; version control for content updates is not required
- **Integration with LMS**: The textbook is standalone and not integrated with Learning Management Systems (Moodle, Canvas, etc.)

---

## Configuration Requirements

**Security Requirement**: All API keys, database connection strings, and authentication secrets MUST be loaded from environment variables. Hard-coding credentials in source code is prohibited.

**Required Environment Variables**:
- `OPENAI_API_KEY`: OpenAI API key for embeddings (text-embedding-3-small) and chat completions (gpt-3.5-turbo or gpt-4)
- `NEON_DATABASE_URL`: Neon Serverless Postgres connection string (format: `postgresql://user:password@host.region.neon.tech/dbname?sslmode=require`)
- `QDRANT_URL`: Qdrant Cloud REST API URL
- `QDRANT_API_KEY`: Qdrant Cloud API key
- `BETTER_AUTH_SECRET`: Secret key for Better-auth JWT token signing (minimum 32 characters, randomly generated)

## Dependencies

- **Docusaurus 3.x**: Static site generator for the textbook
- **React 19.x**: UI library for chatbot component and interactive elements
- **FastAPI**: Python backend framework for API endpoints
- **OpenAI API**: Embeddings (text-embedding-3-small) and chat completions (gpt-3.5-turbo or gpt-4) via Agents SDK and ChatKit SDK
- **Qdrant Cloud Free Tier**: Vector database for storing textbook content embeddings and performing similarity search
- **Neon Serverless Postgres**: Database for storing user profiles, conversation history, translation cache, and session data
- **Better-auth** (https://www.better-auth.com/): Modern TypeScript-first authentication library for signup/signin functionality with JWT token management
- **tiktoken**: Token counting library for accurate text chunking
- **GitHub Pages**: Hosting platform for deployed textbook
- **Cloud hosting for backend**: Render, Railway, Fly.io, or Vercel for deploying FastAPI backend
- **CORS configuration**: Backend must allow requests from GitHub Pages domain

---

## Notes for Implementation

- **Content creation priority**: Focus on comprehensive coverage of ROS 2 (Module 1) and NVIDIA Isaac (Module 3) as these are core to the course; Gazebo/Unity (Module 2) and VLA (Module 4) can be more concise if time-constrained
- **Reuse existing infrastructure**: The RAG pipeline, chatbot component, and agent skills from the previous hackathon project (001-hackathon-app) can be adapted with minimal changes—only textbook content and Neon Postgres integration are new
- **Database schema design**: Neon Postgres should have tables for users (id, email, password_hash, software_background JSON, hardware_background JSON, created_at), conversations (id, user_id, session_id, question, answer, citations JSON, timestamp, question_type)
- **Better-auth integration**: Use Better-auth with JWT tokens. For production, tokens MUST be stored in httpOnly cookies (with localStorage fallback for development). Include token in Authorization header for authenticated API requests. Protected routes MUST validate JWT tokens and return 401 Unauthorized for invalid or expired tokens.
- **Background questionnaire UX**: Present questionnaire immediately after email/password signup, before redirecting to textbook; store responses in JSON columns for flexibility
- **Agent skills reusability**: The P+Q+P pattern should be documented in markdown files alongside skill implementations, enabling future contributors to create new skills following the same pattern
- **Demo video strategy**: Focus on showing textbook navigation, asking a question via RAG, using selected-text mode, and signing up with background questionnaire—target 90 seconds exactly as judges only watch the first 90 seconds
