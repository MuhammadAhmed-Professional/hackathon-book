# Implementation Plan: Hackathon App - Docusaurus Book with RAG Chatbot

**Branch**: `001-hackathon-app` | **Date**: 2025-01-27 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-hackathon-app/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

This project implements an integrated hackathon application consisting of: (1) A Docusaurus-based educational book about AI-Driven and Spec-Driven Development deployed to GitHub Pages, (2) A FastAPI backend with RAG capabilities using Qdrant Cloud and OpenAI, (3) A React chatbot UI embedded in the Docusaurus site, (4) Selected-text question mode for focused queries, and (5) Reusable agent skills following the Persona + Questions + Principles pattern. The technical approach uses Docusaurus for documentation, FastAPI for the backend API, Qdrant Cloud Free Tier for vector storage, and OpenAI Agents SDK/ChatKit SDK for embeddings and chat completions.

## Technical Context

**Language/Version**: 
- Python 3.11+ (for FastAPI backend)
- Node.js 18+ (for Docusaurus frontend)
- JavaScript/TypeScript (for React components)

**Primary Dependencies**: 
- Frontend: Docusaurus, React, React DOM
- Backend: FastAPI, uvicorn, qdrant-client, openai (Agents SDK, ChatKit SDK), python-dotenv, tiktoken
- Vector Database: Qdrant Cloud Free Tier (hosted service)
- AI Services: OpenAI API (embeddings and chat completions)

**Storage**: 
- Qdrant Cloud Free Tier for vector embeddings and metadata
- Local file system for Docusaurus book content (markdown files)
- Environment variables for API keys and configuration

**Testing**: 
- Backend: pytest, httpx (for FastAPI testing)
- Frontend: Jest, React Testing Library (optional, for React component testing)
- Manual testing for integration and user acceptance

**Target Platform**: 
- Web browsers (Chrome, Firefox, Safari, Edge - modern versions)
- GitHub Pages for book deployment
- Cloud-hosted FastAPI backend (or local development server)

**Project Type**: Web application (frontend + backend)

**Performance Goals**: 
- Book pages load in under 2 seconds
- RAG query responses in under 5 seconds (90% of requests)
- Chatbot UI renders and responds without noticeable lag
- Support concurrent users (hackathon scale: 10-50 simultaneous users)

**Constraints**: 
- Qdrant Cloud Free Tier limits (collection size, API rate limits)
- OpenAI API rate limits and costs
- GitHub Pages deployment constraints (static site generation)
- Must work on Windows development environment
- All secrets must be environment-based (no hard-coded credentials)

**Scale/Scope**: 
- Book: 7 chapters with educational content
- Vector chunks: Estimated 200-500 chunks depending on book length
- API endpoints: 3-4 endpoints (health, ingest, ask, ask_selected)
- React components: 1-2 main components (chatbot, selected-text handler)
- Agent skills: 3 skills (SummarizeSection, GenerateQuizQuestions, ExplainTerm)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Pre-Research Gates

✅ **Spec-Driven Development**: Plan follows Spec-Kit Plus workflow (Constitution → Specify → Plan → Tasks → Implement)

✅ **Reusable Intelligence**: Plan will generate ADRs for major decisions (Docusaurus choice, FastAPI choice, RAG strategy, chunking approach)

✅ **AI-Driven Development**: Plan uses AI tools (Claude Code) for code generation based on specifications

✅ **Educational Clarity**: Plan prioritizes simple, readable, well-commented code suitable for student learning

✅ **Security and Secrets**: Plan uses environment variables for all API keys (OpenAI, Qdrant) - no hard-coded credentials

✅ **Architecture Decision Documentation**: Plan includes ADR creation for framework choices, RAG strategy, chunking strategy, deployment approach

### Post-Design Gates (to be re-evaluated after Phase 1)

- [ ] Architecture decisions documented in ADRs
- [ ] No hard-coded secrets in code structure
- [ ] Code structure supports simple, educational implementations
- [ ] All major technology choices justified in ADRs

## Project Structure

### Documentation (this feature)

```text
specs/001-hackathon-app/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   ├── openapi.yaml     # API contract for FastAPI backend
│   └── README.md        # Contract documentation
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Web application structure (frontend + backend)

backend/
├── src/
│   ├── models/          # Data models (chunk, question, answer)
│   ├── services/         # Business logic (rag_service, ingestion_service, skills)
│   ├── api/             # FastAPI routes and endpoints
│   └── config.py        # Configuration and environment variables
├── tests/
│   ├── unit/            # Unit tests for services and models
│   ├── integration/     # Integration tests for API endpoints
│   └── contract/       # Contract tests
└── requirements.txt     # Python dependencies

frontend/
├── docs/                # Docusaurus book content (markdown files)
│   ├── intro.md
│   ├── ai-driven-development.md
│   ├── spec-driven-development.md
│   ├── rag-fundamentals.md
│   ├── implementation-guide.md
│   ├── chatbot-usage.md
│   └── future-work.md
├── src/
│   ├── components/      # React components
│   │   ├── Chatbot.tsx  # Main chatbot component
│   │   └── SelectedTextHandler.tsx  # Selected-text mode handler
│   └── css/            # Custom styles
├── static/             # Static assets
├── docusaurus.config.js  # Docusaurus configuration
├── package.json        # Node.js dependencies
└── babel.config.js     # Babel configuration

.env.example            # Example environment variables template
.gitignore             # Git ignore rules
README.md              # Project documentation
```

**Structure Decision**: Web application structure selected because the project has distinct frontend (Docusaurus/React) and backend (FastAPI) components. This separation allows independent development, testing, and deployment. The frontend contains the Docusaurus book content and React chatbot components, while the backend handles RAG operations, vector database interactions, and agent skills.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations identified. The architecture follows constitution principles:
- Simple, educational code structure
- Clear separation of concerns (frontend/backend)
- Environment-based configuration
- Documented architecture decisions (via ADRs)

