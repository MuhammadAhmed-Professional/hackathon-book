# AI-Driven and Spec-Driven Faculty Hackathon

An integrated hackathon application demonstrating AI-Driven and Spec-Driven Development practices. The project consists of:

1. **Docusaurus Book**: An educational book about AI-Driven Development, Spec-Driven Development, RAG fundamentals, and implementation guide, deployed to GitHub Pages
2. **RAG Chatbot**: A FastAPI backend with RAG capabilities using Qdrant Cloud and OpenAI, with a React chatbot UI embedded in the Docusaurus site
3. **Selected-Text Mode**: Users can select text in the book and ask questions about only that selected context
4. **Reusable Agent Skills**: Three agent skills (SummarizeSection, GenerateQuizQuestions, ExplainTerm) following the Persona + Questions + Principles pattern

## Quick Start

See [specs/001-hackathon-app/quickstart.md](specs/001-hackathon-app/quickstart.md) for detailed setup and running instructions.

## Project Structure

```
.
├── backend/          # FastAPI backend application
│   ├── src/         # Source code (models, services, API endpoints)
│   └── requirements.txt
├── frontend/        # Docusaurus documentation site
│   ├── docs/        # Book chapters (markdown files)
│   ├── src/         # React components (chatbot, selected-text handler)
│   └── docusaurus.config.js
└── specs/           # Spec-Kit Plus specifications
    └── 001-hackathon-app/
        ├── spec.md   # Feature specification
        ├── plan.md   # Implementation plan
        └── tasks.md  # Task breakdown
```

## Technology Stack

- **Frontend**: Docusaurus, React
- **Backend**: FastAPI, Python 3.11+
- **Vector Database**: Qdrant Cloud Free Tier
- **AI Services**: OpenAI (Agents SDK, ChatKit SDK)
- **Deployment**: GitHub Pages

## Live Deployment

🌐 **Live Site**: https://MuhammadAhmed-Professional.github.io/hackathon-book/

## Development

### Prerequisites

- Python 3.11+
- Node.js 18+
- OpenAI API key
- Qdrant Cloud account (Free Tier)

### Setup

1. **Backend Setup**:
   ```bash
   cd backend
   python -m venv venv
   venv\Scripts\activate  # Windows
   pip install -r requirements.txt
   ```

2. **Frontend Setup**:
   ```bash
   cd frontend
   npm install
   ```

3. **Configuration**:
   - Copy `backend/.env.example` to `backend/.env` (if not blocked)
   - Add your OpenAI API key and Qdrant credentials

### Running

- **Backend**: `cd backend && uvicorn src.api.main:app --reload`
- **Frontend**: `cd frontend && npm start`

## Documentation

- [Specification](specs/001-hackathon-app/spec.md)
- [Implementation Plan](specs/001-hackathon-app/plan.md)
- [Quick Start Guide](specs/001-hackathon-app/quickstart.md)

## License

This project is part of the AI-Driven and Spec-Driven Faculty Hackathon.

