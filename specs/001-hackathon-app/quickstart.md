# Quickstart Guide: Hackathon App

**Date**: 2025-01-27  
**Feature**: 001-hackathon-app

This guide provides step-by-step instructions to get the hackathon app running locally.

## Prerequisites

- Python 3.11+ installed
- Node.js 18+ and npm installed
- Git installed
- OpenAI API key
- Qdrant Cloud account (Free Tier) with API key and URL

## Step 1: Clone and Setup Repository

```bash
git clone <repository-url>
cd hackathon
git checkout 001-hackathon-app
```

## Step 2: Backend Setup

### 2.1 Create Virtual Environment

```bash
cd backend
python -m venv venv

# Windows
venv\Scripts\activate

# Linux/Mac
source venv/bin/activate
```

### 2.2 Install Dependencies

```bash
pip install -r requirements.txt
```

### 2.3 Configure Environment Variables

Create `.env` file in `backend/` directory:

```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
```

**Important**: Never commit `.env` file to git. It's already in `.gitignore`.

### 2.4 Run Backend Server

```bash
uvicorn src.api.main:app --reload --port 8000
```

Backend should be running at `http://localhost:8000`

Verify with: `curl http://localhost:8000/ping`

## Step 3: Frontend Setup

### 3.1 Install Dependencies

```bash
cd frontend
npm install
```

### 3.2 Configure Backend URL

Update `frontend/src/config.js` (or environment variable) to point to backend:

```javascript
export const API_BASE_URL = 'http://localhost:8000';
```

### 3.3 Run Development Server

```bash
npm start
```

Frontend should be running at `http://localhost:3000`

## Step 4: Ingest Book Content

Before using the chatbot, ingest book content into Qdrant:

```bash
# From backend directory
curl -X POST http://localhost:8000/ingest
```

Wait for ingestion to complete (may take a few minutes depending on book size).

Verify ingestion:
- Check Qdrant Cloud dashboard for collection and vector count
- Or test with a question: `curl -X POST http://localhost:8000/ask -H "Content-Type: application/json" -d '{"question": "What is RAG?"}'`

## Step 5: Test the Application

### 5.1 Test Book Navigation

1. Open `http://localhost:3000` in browser
2. Navigate through chapters using sidebar
3. Verify all internal links work

### 5.2 Test RAG Chatbot

1. Open chatbot interface (embedded in book pages)
2. Ask a question: "What is AI-Driven Development?"
3. Verify answer is returned with source citations

### 5.3 Test Selected-Text Mode

1. Select/highlight text in any chapter
2. Click "Ask about this selection" button
3. Ask a question about the selected text
4. Verify answer uses only selected context

## Troubleshooting

### Backend Issues

**Error: "OPENAI_API_KEY not found"**
- Check `.env` file exists in `backend/` directory
- Verify environment variable name matches exactly
- Restart the server after creating/updating `.env`

**Error: "Qdrant connection failed"**
- Verify Qdrant URL and API key in `.env`
- Check Qdrant Cloud dashboard for cluster status
- Ensure network connectivity

**Error: "Module not found"**
- Activate virtual environment: `venv\Scripts\activate` (Windows)
- Reinstall dependencies: `pip install -r requirements.txt`

### Frontend Issues

**Error: "Cannot connect to backend"**
- Verify backend is running on port 8000
- Check CORS configuration in FastAPI
- Verify API_BASE_URL in frontend config

**Error: "Docusaurus build failed"**
- Clear cache: `npm run clear`
- Reinstall dependencies: `rm -rf node_modules && npm install`
- Check Node.js version: `node --version` (should be 18+)

### Ingestion Issues

**Error: "No chunks ingested"**
- Verify book markdown files exist in `frontend/docs/`
- Check file permissions
- Review backend logs for specific errors

**Error: "Embedding generation failed"**
- Verify OpenAI API key is valid
- Check API rate limits
- Review OpenAI account status

## Next Steps

After local setup is working:

1. **Deploy Frontend to GitHub Pages**: Follow Docusaurus deployment guide
2. **Deploy Backend**: Choose hosting (Render, Fly.io, Railway) or keep local for demo
3. **Test End-to-End**: Verify all features work in deployed environment
4. **Documentation**: Update this guide with any environment-specific notes

## Development Workflow

1. Make changes to code
2. Test locally (backend + frontend)
3. Commit changes: `git commit -m "feat: description"`
4. Push to branch: `git push origin 001-hackathon-app`
5. Create PR when ready for review

## Resources

- [Docusaurus Documentation](https://docusaurus.io/docs)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [Qdrant Cloud Documentation](https://qdrant.tech/documentation/)
- [OpenAI API Documentation](https://platform.openai.com/docs)

