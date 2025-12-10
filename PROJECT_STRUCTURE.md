# Project Structure - Physical AI & Humanoid Robotics Textbook

## Executive Summary

This is a **monorepo full-stack educational platform** combining a Docusaurus-based interactive textbook with a FastAPI backend providing RAG (Retrieval-Augmented Generation) capabilities and agent skills. The structure follows industry-standard practices for separation of concerns, scalability, and maintainability.

**Architecture Philosophy:**
- Clear separation between frontend presentation and backend services
- Spec-Driven Development with comprehensive documentation
- Feature-based organization with co-located concerns
- Docker-ready for containerized deployment
- Test directories mirror source structure

---

## Complete Directory Tree

```
D:\Talal\Work\Hackathon\
├── .claude/                          # Claude Code CLI configuration
│   ├── agents/                       # Custom agent definitions
│   └── commands/                     # Custom slash commands
│
├── .git/                             # Git version control (ignored in docs)
│
├── .specify/                         # Spec-Kit Plus framework
│   ├── memory/                       # Project constitution and memory
│   │   ├── constitution.md          # Project principles & standards
│   │   └── image/                   # Constitution reference images
│   │       └── constitution/
│   ├── scripts/                      # Automation scripts (if needed)
│   └── skills/                       # Agent skill definitions
│       ├── better-auth-validator.md
│       ├── content-personalizer.md
│       ├── explain-term.md
│       ├── generate-quiz-questions.md
│       ├── robotics-chapter-writer.md
│       ├── selected-text-validator.md
│       ├── summarize-section.md
│       ├── validate-chapter-structure.md
│       └── write-textbook-chapter.md
│
├── backend/                          # FastAPI backend application
│   ├── src/                          # Source code
│   │   ├── api/                      # API layer
│   │   │   ├── endpoints/            # Route handlers (versioned)
│   │   │   │   ├── ask.py           # RAG question endpoint
│   │   │   │   ├── ask_selected.py  # Selected-text RAG endpoint
│   │   │   │   ├── auth.py          # Authentication endpoints
│   │   │   │   ├── ingest.py        # Document ingestion endpoint
│   │   │   │   ├── personalization.py # Personalization endpoints
│   │   │   │   ├── ping.py          # Health check endpoint
│   │   │   │   ├── skills.py        # Agent skills endpoint
│   │   │   │   └── __init__.py
│   │   │   ├── dependencies.py      # FastAPI dependencies
│   │   │   ├── main.py              # FastAPI app entry point
│   │   │   └── __init__.py
│   │   ├── db/                       # Database layer
│   │   │   ├── connection.py        # Neon Postgres connection
│   │   │   ├── migrations/          # Database migrations
│   │   │   └── __init__.py
│   │   ├── models/                   # Data models (Pydantic)
│   │   │   ├── answer.py            # Answer response model
│   │   │   ├── chunk.py             # Document chunk model
│   │   │   ├── conversation.py      # Conversation model
│   │   │   ├── question.py          # Question request model
│   │   │   ├── source_citation.py   # Source citation model
│   │   │   ├── user.py              # User model
│   │   │   └── __init__.py
│   │   ├── services/                 # Business logic layer
│   │   │   ├── skills/               # Agent skills implementations
│   │   │   │   ├── explain_term.py  # Explain terminology skill
│   │   │   │   ├── generate_quiz.py # Generate quiz questions skill
│   │   │   │   ├── summarize_section.py # Summarize section skill
│   │   │   │   └── __init__.py
│   │   │   ├── agent_skills_service.py # Agent skills orchestration
│   │   │   ├── auth_service.py      # Authentication service
│   │   │   ├── db_service.py        # Database service
│   │   │   ├── ingestion_service.py # Document ingestion service
│   │   │   ├── personalization_service.py # Personalization service
│   │   │   └── rag_service.py       # RAG service (OpenAI + Qdrant)
│   │   ├── config.py                 # Configuration management
│   │   └── __init__.py
│   ├── tests/                        # [TODO] Test suite
│   │   ├── unit/                     # Unit tests
│   │   ├── integration/              # Integration tests
│   │   └── conftest.py               # Pytest configuration
│   ├── .env.example                  # Environment variables template
│   ├── Dockerfile                    # [TODO] Docker image definition
│   ├── requirements.txt              # Python dependencies
│   └── README.md                     # [TODO] Backend documentation
│
├── frontend/                         # Docusaurus documentation site
│   ├── docs/                         # Markdown content (textbook chapters)
│   │   ├── module1/                  # Module 1: ROS 2 Fundamentals
│   │   ├── module2/                  # Module 2: Gazebo & Unity Simulation
│   │   ├── module3/                  # Module 3: NVIDIA Isaac Platform
│   │   ├── module4/                  # Module 4: Vision-Language-Action
│   │   ├── review/                   # Review & Practice Chapters
│   │   ├── tutorial-basics/          # Docusaurus tutorial samples
│   │   ├── tutorial-extras/          # Docusaurus tutorial extras
│   │   ├── ai-driven-development.md
│   │   ├── chatbot-usage.md
│   │   ├── future-work.md
│   │   ├── hardware.md
│   │   ├── implementation-guide.md
│   │   ├── intro.md
│   │   ├── rag-fundamentals.md
│   │   ├── spec-driven-development.md
│   │   └── weekly-breakdown.md
│   ├── src/                          # React components & custom code
│   │   ├── components/               # Reusable React components
│   │   │   ├── HomepageFeatures/    # Homepage feature cards
│   │   │   ├── AuthForms.css
│   │   │   ├── AuthProvider.tsx     # Authentication context provider
│   │   │   ├── Chatbot.css
│   │   │   ├── Chatbot.tsx          # Main chatbot component
│   │   │   ├── ChatbotContext.tsx   # Chatbot state management
│   │   │   ├── FloatingChatbot.css
│   │   │   ├── FloatingChatbot.tsx  # Floating chatbot UI
│   │   │   ├── PersonalizedContent.css
│   │   │   ├── PersonalizedContent.tsx # Personalized content component
│   │   │   ├── ProfileButton.css
│   │   │   ├── ProfileButton.tsx    # User profile button
│   │   │   ├── SelectedTextHandler.tsx # Selected-text mode handler
│   │   │   ├── SigninForm.tsx       # Sign-in form
│   │   │   └── SignupForm.tsx       # Sign-up form
│   │   ├── css/                      # Global stylesheets
│   │   │   └── custom.css
│   │   ├── lib/                      # Utility libraries
│   │   │   └── auth.ts              # Better-auth client setup
│   │   ├── pages/                    # Custom pages
│   │   │   ├── index.js             # Homepage
│   │   │   ├── index.module.css
│   │   │   ├── markdown-page.md
│   │   │   ├── signin.tsx           # Sign-in page
│   │   │   └── signup.tsx           # Sign-up page
│   │   ├── theme/                    # Docusaurus theme customizations
│   │   │   ├── DocItem/Layout/
│   │   │   ├── Navbar/Content/
│   │   │   └── Root.tsx             # Root theme wrapper
│   │   └── config.js                 # Frontend configuration
│   ├── static/                       # Static assets
│   │   └── img/                      # Images
│   ├── tests/                        # [TODO] Frontend tests
│   │   └── __tests__/                # Jest test files
│   ├── .gitignore                    # Frontend-specific ignores (in root)
│   ├── docusaurus.config.js          # Docusaurus configuration
│   ├── Dockerfile                    # [TODO] Docker image definition
│   ├── package.json                  # Node.js dependencies
│   ├── sidebars.js                   # Sidebar navigation structure
│   └── README.md                     # [TODO] Frontend documentation
│
├── history/                          # Historical records & decisions
│   ├── adr/                          # Architecture Decision Records
│   │   ├── agent-skills-design.md
│   │   ├── deployment-approach.md
│   │   ├── docusaurus-choice.md
│   │   ├── fastapi-choice.md
│   │   └── rag-strategy.md
│   └── prompts/                      # Prompt History Records (PHRs)
│       ├── 001-hackathon-app/       # Feature 001 prompts
│       ├── 002-physical-ai-book/    # Feature 002 prompts
│       ├── constitution/            # Constitution-related prompts
│       └── general/                 # General prompts
│
├── specs/                            # Spec-Kit Plus specifications
│   ├── 001-hackathon-app/           # Feature 001: Hackathon App
│   │   ├── checklists/              # Quality checklists
│   │   ├── contracts/               # API contracts (OpenAPI)
│   │   ├── analysis-report.md
│   │   ├── data-model.md
│   │   ├── plan.md                  # Implementation plan
│   │   ├── quickstart.md            # Quick start guide
│   │   ├── research.md              # Research & background
│   │   ├── spec.md                  # Feature specification
│   │   └── tasks.md                 # Task breakdown
│   └── 002-physical-ai-book/        # Feature 002: Physical AI Book
│       ├── checklists/
│       ├── contracts/
│       ├── data-model.md
│       ├── plan.md
│       ├── quickstart.md
│       ├── research.md
│       ├── spec.md
│       └── tasks.md
│
├── venv/                             # Python virtual environment (should be ignored)
│
├── .gitignore                        # Git ignore patterns
├── CLAUDE.md                         # Claude Code rules & conventions
├── docker-compose.yml                # [TODO] Local development orchestration
├── IMPLEMENTATION_PROGRESS.md        # Implementation progress tracking
├── PROJECT_STRUCTURE.md              # This file
└── README.md                         # Project overview & setup

---

## Deployment Artifacts (GitHub Pages - in root)

These are generated by `npm run build` and deployed to gh-pages branch:
- 404.html
- index.html
- markdown-page.html
- sitemap.xml
- assets/
- docs/
- img/

**Note:** These should ideally be in a `build/` directory, but GitHub Pages deployment may place them at root.

```

---

## Directory Purposes

### Root Level

| Directory/File | Purpose |
|----------------|---------|
| `.claude/` | Claude Code CLI configuration for agents and custom commands |
| `.specify/` | Spec-Kit Plus framework for specifications, skills, and project constitution |
| `backend/` | FastAPI backend application providing RAG services and API endpoints |
| `frontend/` | Docusaurus static site generator for interactive textbook |
| `history/` | Architecture Decision Records (ADRs) and Prompt History Records (PHRs) |
| `specs/` | Feature specifications following Spec-Kit Plus methodology |
| `venv/` | Python virtual environment (should be in .gitignore) |
| `.gitignore` | Git ignore patterns for build artifacts, dependencies, secrets |
| `CLAUDE.md` | Claude Code operational rules and development guidelines |
| `docker-compose.yml` | [MISSING] Local development environment orchestration |
| `README.md` | Project overview, tech stack, and setup instructions |

### Backend Structure

| Directory | Purpose |
|-----------|---------|
| `src/api/` | API layer with route handlers and FastAPI app initialization |
| `src/api/endpoints/` | Individual endpoint modules (ask, auth, ingest, skills, etc.) |
| `src/db/` | Database connection and migrations for Neon Postgres |
| `src/models/` | Pydantic data models for request/response validation |
| `src/services/` | Business logic layer (RAG, authentication, ingestion, personalization) |
| `src/services/skills/` | Agent skill implementations (explain, quiz, summarize) |
| `tests/` | [MISSING] Test suite mirroring src/ structure |
| `.env.example` | Environment variable template with placeholders |
| `requirements.txt` | Python dependencies (FastAPI, OpenAI, Qdrant, etc.) |

### Frontend Structure

| Directory | Purpose |
|-----------|---------|
| `docs/` | Markdown content for textbook chapters organized by module |
| `src/components/` | React components (Chatbot, Auth, PersonalizedContent, etc.) |
| `src/lib/` | Utility libraries and shared logic |
| `src/pages/` | Custom pages (homepage, signin, signup) |
| `src/theme/` | Docusaurus theme customizations and overrides |
| `static/img/` | Static images and assets |
| `tests/` | [MISSING] Frontend test suite |
| `docusaurus.config.js` | Docusaurus site configuration (title, URL, theme, navbar) |
| `sidebars.js` | Sidebar navigation structure |
| `package.json` | Node.js dependencies and scripts |

### Spec-Kit Plus Structure

| Directory | Purpose |
|-----------|---------|
| `.specify/memory/` | Project constitution defining principles and standards |
| `.specify/skills/` | Agent skill definitions for content generation and validation |
| `specs/<feature>/` | Feature-specific specifications, plans, and tasks |
| `specs/<feature>/checklists/` | Quality and validation checklists |
| `specs/<feature>/contracts/` | API contracts in OpenAPI format |
| `history/adr/` | Architecture Decision Records documenting significant decisions |
| `history/prompts/` | Prompt History Records tracking AI-assisted development sessions |

---

## Naming Conventions

### Directories

- **Lowercase with hyphens** for multi-word directories: `user-profile/`, `api-client/`
- **Plural for collections**: `components/`, `services/`, `models/`, `endpoints/`, `skills/`
- **Singular for single-purpose**: `config/`, `middleware/`, `api/`

### Files

**Python (Backend):**
- **snake_case** for all files: `rag_service.py`, `auth_service.py`, `user_model.py`
- **Test files**: `test_rag_service.py`, `test_auth.py`

**JavaScript/TypeScript (Frontend):**
- **PascalCase for React components**: `Chatbot.tsx`, `FloatingChatbot.tsx`, `AuthProvider.tsx`
- **camelCase for utilities**: `apiClient.ts`, `formatDate.ts`
- **Hooks**: `useAuth.ts`, `useChatBot.ts`
- **CSS modules**: `Chatbot.css`, `ProfileButton.css`

**Configuration Files:**
- **kebab-case with extensions**: `docker-compose.yml`, `docusaurus.config.js`, `.env.example`
- **Dotfiles**: `.gitignore`, `.eslintrc`, `.prettierrc`

**Markdown Documentation:**
- **kebab-case**: `implementation-guide.md`, `rag-fundamentals.md`, `weekly-breakdown.md`

---

## Setup Instructions

### Prerequisites

- **Python**: 3.11+ (for backend)
- **Node.js**: 20.0+ (for frontend)
- **Git**: Latest version
- **OpenAI API Key**: For RAG functionality
- **Qdrant Cloud Account**: Free tier for vector database
- **Neon Serverless Postgres**: Free tier for user data

### 1. Clone Repository

```bash
git clone https://github.com/talal/Hackathon.git
cd Hackathon
```

### 2. Backend Setup

```bash
cd backend

# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# macOS/Linux:
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt

# Configure environment variables
cp .env.example .env
# Edit .env with your API keys:
# - OPENAI_API_KEY
# - QDRANT_URL
# - QDRANT_API_KEY
# - NEON_DATABASE_URL
# - BETTER_AUTH_SECRET
```

### 3. Frontend Setup

```bash
cd frontend

# Install dependencies
npm install

# Start development server
npm start
```

### 4. Running the Application

**Terminal 1 - Backend:**
```bash
cd backend
venv\Scripts\activate  # Windows
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000
```

**Terminal 2 - Frontend:**
```bash
cd frontend
npm start
```

**Access:**
- Frontend: http://localhost:3000
- Backend API: http://localhost:8000
- API Docs: http://localhost:8000/docs

### 5. Docker Setup (When Available)

```bash
# Build and start all services
docker-compose up --build

# Stop services
docker-compose down
```

---

## Integration Points for Developers & Agents

### For Content Creation Agents

- **Primary Location**: `frontend/docs/`
- **Module Structure**: `frontend/docs/module1/`, `frontend/docs/module2/`, etc.
- **Naming**: Use kebab-case for markdown files (e.g., `ros2-fundamentals.md`)
- **Images**: Place in `frontend/static/img/module1/`, `frontend/static/img/module2/`, etc.
- **Sidebar**: Update `frontend/sidebars.js` when adding new chapters

### For API Development Agents

- **Endpoints**: Create in `backend/src/api/endpoints/`
- **Services**: Add business logic in `backend/src/services/`
- **Models**: Define Pydantic models in `backend/src/models/`
- **Tests**: Mirror structure in `backend/tests/`
- **OpenAPI**: Update contract in `specs/<feature>/contracts/openapi.yaml`

### For UI/UX Agents

- **Components**: Create in `frontend/src/components/` with co-located CSS
- **Pages**: Add custom pages in `frontend/src/pages/`
- **Styling**: Global styles in `frontend/src/css/custom.css`
- **Theme**: Customize in `frontend/src/theme/`

### For Testing Agents

- **Backend Tests**: Create in `backend/tests/` (unit/, integration/)
- **Frontend Tests**: Create in `frontend/tests/__tests__/`
- **Test Naming**: `test_<module>.py` (Python), `<Component>.test.tsx` (React)

### For Deployment Agents

- **Docker**: Create `Dockerfile` in `backend/` and `frontend/`
- **Compose**: Update `docker-compose.yml` at root
- **CI/CD**: Configuration in `.github/workflows/` (if created)
- **Environment**: Never commit `.env` files; use `.env.example` as template

### For Documentation Agents

- **ADRs**: Create in `history/adr/` with format `<topic>.md`
- **Specs**: Use `specs/<feature>/spec.md` for requirements
- **Plans**: Use `specs/<feature>/plan.md` for architecture
- **Tasks**: Use `specs/<feature>/tasks.md` for task breakdown
- **README**: Update root `README.md` and create subdirectory READMEs

---

## Quality Assurance Checklist

### Structural Validation

- [x] Clear separation between frontend and backend
- [x] Configuration files in appropriate locations
- [ ] Test directories mirror source structure (MISSING)
- [x] Documentation is discoverable and organized
- [x] No circular dependencies in folder hierarchy
- [x] Common files (.gitignore, .env.example) present
- [x] Follows framework-specific conventions (Docusaurus, FastAPI)
- [x] Scalable for adding new features/modules
- [x] Tool-friendly (works with bundlers, linters, formatters)

### Missing Components

- [ ] `docker-compose.yml` at root
- [ ] `backend/Dockerfile`
- [ ] `frontend/Dockerfile`
- [ ] `backend/tests/` directory with structure
- [ ] `frontend/tests/` directory with structure
- [ ] `backend/README.md`
- [ ] `frontend/README.md`
- [ ] `.specify/README.md`

### Configuration Quality

- [x] `.gitignore` organized by category
- [x] `.env.example` with clear sections and placeholders
- [x] `package.json` with organized scripts
- [x] `requirements.txt` organized by purpose
- [ ] `pytest.ini` for backend testing (MISSING)
- [ ] `jest.config.js` for frontend testing (MISSING)

---

## Migration Notes

### For Existing Developers

1. **Virtual Environment**: Ensure `venv/` is in `.gitignore` (currently missing)
2. **Build Artifacts**: GitHub Pages deployment artifacts at root are acceptable for this setup
3. **Tests**: Test directories need to be created before writing tests
4. **Docker**: Docker configuration will be added for containerized development

### Proposed Changes

1. Add `venv/` to `.gitignore`
2. Create `backend/tests/` directory structure
3. Create `frontend/tests/` directory structure
4. Add Docker configuration files
5. Create README files for major subdirectories
6. Add test configuration files (pytest.ini, jest.config.js)

---

## Technology Stack Summary

### Frontend
- **Framework**: Docusaurus 3.9.2
- **UI Library**: React 19.0.0
- **Authentication**: better-auth 1.0.0
- **Styling**: CSS Modules + Custom CSS
- **Build Tool**: Webpack (via Docusaurus)

### Backend
- **Framework**: FastAPI 0.109.0
- **Server**: Uvicorn 0.27.0
- **AI/ML**: OpenAI 1.12.0, tiktoken 0.8.0+
- **Vector DB**: Qdrant Client 1.7.1+
- **Database**: PostgreSQL (Neon) via psycopg2-binary 2.9.9+
- **Authentication**: python-jose 3.3.0, passlib 1.7.4
- **Testing**: pytest 7.4.4, httpx 0.26.0

### Infrastructure
- **Version Control**: Git + GitHub
- **Deployment**: GitHub Pages (frontend), Neon Serverless (database), Qdrant Cloud (vectors)
- **Containerization**: Docker (planned)

---

## Validation Against Industry Standards

| Standard | Status | Notes |
|----------|--------|-------|
| Monorepo structure | ✅ PASS | Clear frontend/backend separation |
| Framework conventions | ✅ PASS | Docusaurus docs/, FastAPI app/ structure |
| Configuration management | ✅ PASS | .env.example, proper config files |
| Documentation co-location | ✅ PASS | Specs, ADRs, PHRs well-organized |
| Test organization | ⚠️ PARTIAL | Structure planned but directories missing |
| Docker readiness | ❌ FAIL | Missing Dockerfile and docker-compose.yml |
| Naming consistency | ✅ PASS | Follows language conventions (snake_case, PascalCase) |
| Scalability | ✅ PASS | Feature-based organization supports growth |
| Tool compatibility | ✅ PASS | Works with IDEs, bundlers, linters |
| Security | ✅ PASS | .env in .gitignore, .env.example provided |

---

## Next Steps

### Immediate Priorities

1. **Create Docker Configuration**
   - `docker-compose.yml` at root
   - `backend/Dockerfile`
   - `frontend/Dockerfile`

2. **Create Test Infrastructure**
   - `backend/tests/` with unit/, integration/ subdirectories
   - `frontend/tests/__tests__/` for Jest tests
   - `backend/pytest.ini` configuration
   - `frontend/jest.config.js` configuration

3. **Update .gitignore**
   - Add `venv/` to ignore list
   - Add temporary files (structure.txt, NUL)
   - Add test coverage reports

4. **Create README Files**
   - `backend/README.md` with API documentation
   - `frontend/README.md` with component documentation
   - `.specify/README.md` with Spec-Kit Plus usage

### Long-term Enhancements

- CI/CD pipeline configuration (.github/workflows/)
- API versioning strategy (v1/, v2/ in endpoints/)
- Comprehensive test coverage (unit, integration, e2e)
- Performance monitoring and logging setup
- Production deployment configuration

---

**Last Updated**: 2025-11-29
**Maintainer**: Spec-Kit Plus / Claude Code
**Version**: 1.0.0
