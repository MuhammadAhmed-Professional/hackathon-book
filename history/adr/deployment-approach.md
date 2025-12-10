# ADR-004: Deployment Approach for GitHub Pages and Backend

**Status**: Accepted  
**Date**: 2025-01-27  
**Context**: Need to deploy both the Docusaurus book (frontend) and FastAPI backend for the hackathon. Must consider cost, ease of deployment, and integration between frontend and backend.

## Decision

We will use:
- **Frontend**: GitHub Pages for Docusaurus book deployment
- **Backend**: Local development server or cloud hosting (Render/Railway/Fly.io) for FastAPI API
- **CORS**: Configured to allow frontend-backend communication

## Rationale

### Frontend Deployment (GitHub Pages)
1. **Free Hosting**: GitHub Pages provides free static site hosting
2. **Integrated with Git**: Automatic deployment from repository
3. **Docusaurus Support**: Built-in GitHub Pages deployment workflow
4. **HTTPS**: Automatic SSL certificates
5. **Custom Domain**: Optional custom domain support

### Backend Deployment Options
1. **Local Development**: FastAPI runs on localhost:8000 for development
2. **Cloud Hosting**: Render/Railway/Fly.io for production (free tiers available)
3. **CORS Configuration**: Allows frontend to call backend API

## Consequences

**Positive**:
- Free hosting for frontend
- Simple deployment workflow
- Good for hackathon demonstration
- Easy to update content

**Negative**:
- Backend requires separate hosting (not included in GitHub Pages)
- CORS configuration needed for cross-origin requests
- GitHub Pages only serves static content (no server-side processing)

## Alternatives Considered

- **Vercel/Netlify**: Alternative static hosting, but GitHub Pages is simpler for GitHub repos
- **Docker + Cloud Run**: More complex but more control
- **Full-stack Platform (Railway)**: Could host both, but separates concerns less clearly

## Deployment Steps

1. **Frontend**:
   - Configure `docusaurus.config.js` with GitHub Pages settings
   - Push to repository
   - Enable GitHub Pages in repository settings
   - Deploy via GitHub Actions or manual build

2. **Backend**:
   - Set environment variables (OpenAI API key, Qdrant credentials)
   - Deploy to cloud hosting service
   - Update frontend `config.js` with backend URL
   - Configure CORS on backend

## References

- GitHub Pages Documentation: https://docs.github.com/en/pages
- Docusaurus Deployment: https://docusaurus.io/docs/deployment#github-pages
- FastAPI Deployment: https://fastapi.tiangolo.com/deployment/

