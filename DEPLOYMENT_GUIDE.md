# Deployment Guide - Physical AI Textbook

## Quick Start

### Prerequisites
- Node.js 18+ (for frontend)
- Python 3.11+ (for backend)
- Git
- OpenAI API key
- Qdrant account (vector database)
- Neon Postgres account (database)

## Frontend Deployment (Docusaurus)

### Option 1: GitHub Pages (Recommended)

1. **Configure repository settings**:
   - Go to Settings → Pages
   - Source: GitHub Actions

2. **Push to gh-pages branch**:
   ```bash
   git push origin gh-pages
   ```

3. **Automatic deployment**:
   - `.github/workflows/deploy.yml` will run automatically
   - Site will be live at `https://yourusername.github.io/repo-name`

### Option 2: Vercel

1. **Install Vercel CLI**:
   ```bash
   npm i -g vercel
   ```

2. **Deploy**:
   ```bash
   cd frontend
   vercel --prod
   ```

3. **Configure**:
   - Framework: Docusaurus
   - Build command: `npm run build`
   - Output directory: `build`

## Backend Deployment (FastAPI)

### Option 1: Railway (Recommended)

1. **Create account**: https://railway.app

2. **Install Railway CLI**:
   ```bash
   npm i -g @railway/cli
   ```

3. **Login and deploy**:
   ```bash
   cd backend
   railway login
   railway init
   railway up
   ```

4. **Set environment variables** in Railway dashboard:
   ```
   BETTER_AUTH_SECRET=<generate-random-string>
   OPENAI_API_KEY=<your-key>
   QDRANT_URL=<your-qdrant-url>
   QDRANT_API_KEY=<your-key>
   DATABASE_URL=<neon-postgres-url>
   ```

5. **Generate deployment domain**:
   - Railway will provide a URL like: `https://your-app.railway.app`

### Option 2: Render

1. **Create account**: https://render.com

2. **Create new Web Service**:
   - Connect GitHub repository
   - Root directory: `backend`
   - Build command: `pip install -r requirements.txt`
   - Start command: `uvicorn src.api.main:app --host 0.0.0.0 --port $PORT`

3. **Set environment variables** in Render dashboard

### Option 3: Vercel (Serverless)

1. **Deploy**:
   ```bash
   cd backend
   vercel --prod
   ```

2. **Configure vercel.json** (already created):
   ```json
   {
     "builds": [{ "src": "src/api/main.py", "use": "@vercel/python" }],
     "routes": [{ "src": "/(.*)", "dest": "src/api/main.py" }]
   }
   ```

## Database Setup

### Neon Postgres

1. **Create account**: https://neon.tech
2. **Create database**: Physical AI Textbook
3. **Copy connection string**:
   ```
   postgresql://user:password@host/database?sslmode=require
   ```
4. **Run migrations** (if any):
   ```bash
   # Tables are created automatically on first run
   ```

### Qdrant Vector Database

1. **Create account**: https://cloud.qdrant.io
2. **Create cluster**: Physical AI Textbook
3. **Get API credentials**:
   - URL: `https://xxx.qdrant.io:6333`
   - API Key: From dashboard
4. **Ingest documentation**:
   ```bash
   curl -X POST https://your-backend.railway.app/ingest
   ```

## Environment Variables

### Backend (.env)
```bash
# Required
BETTER_AUTH_SECRET=minimum-32-characters-random-string
OPENAI_API_KEY=sk-...
QDRANT_URL=https://...
QDRANT_API_KEY=...
NEON_DATABASE_URL=postgresql://...

# Optional
PORT=8000
CORS_ORIGINS=https://yourdomain.com
```

### Frontend (config.ts)
```typescript
export default {
  API_BASE_URL: process.env.REACT_APP_API_URL || 'https://your-backend.railway.app'
}
```

## CI/CD Setup

### GitHub Actions (Automatic)

1. **Test workflow** (`.github/workflows/test.yml`):
   - Runs on every push/PR
   - Tests backend + frontend
   - Must pass before merge

2. **Deploy workflow** (`.github/workflows/deploy.yml`):
   - Runs on push to `gh-pages`
   - Deploys frontend to GitHub Pages
   - Backend deployment (configure as needed)

### Secrets Configuration

Add to GitHub repository secrets (Settings → Secrets):
```
BETTER_AUTH_SECRET
OPENAI_API_KEY
QDRANT_URL
QDRANT_API_KEY
NEON_DATABASE_URL
RAILWAY_TOKEN (if using Railway)
RENDER_API_KEY (if using Render)
```

## Post-Deployment

### 1. Verify Deployment

```bash
# Test backend health
curl https://your-backend.railway.app/ping

# Test frontend
curl https://yourusername.github.io/repo-name
```

### 2. Ingest Documentation

```bash
curl -X POST https://your-backend.railway.app/ingest
```

### 3. Test RAG Chatbot

Visit frontend and ask: "What is ROS 2?"

### 4. Monitor Logs

**Railway**: Dashboard → Deployments → Logs
**Render**: Dashboard → Logs
**GitHub Pages**: Actions tab

## Updating Deployment

### Frontend Updates

```bash
cd frontend
git add .
git commit -m "Update content"
git push origin gh-pages  # Triggers auto-deploy
```

### Backend Updates

```bash
cd backend
git add .
git commit -m "Update backend"
git push origin gh-pages

# Railway/Render will auto-deploy from git
```

## Troubleshooting

### Backend not starting
- Check environment variables are set
- Check logs: `railway logs` or Render dashboard
- Verify NEON_DATABASE_URL is accessible

### Frontend build fails
- Clear node_modules: `rm -rf node_modules && npm install`
- Check Node.js version: `node -v` (should be 18+)

### Database connection fails
- Verify NEON_DATABASE_URL format
- Check Neon database is running
- Ensure IP whitelisting allows Railway/Render

### CORS errors
- Add frontend URL to backend CORS_ORIGINS
- Redeploy backend

## Cost Estimates

### Free Tier (Sufficient for Hackathon)
- **Frontend**: GitHub Pages (free)
- **Backend**: Railway free tier ($5 credit)
- **Database**: Neon free tier (3GB)
- **Vector DB**: Qdrant free tier (1GB)
- **OpenAI**: Pay per use (~$0.50 for demo)

**Total**: ~$0.50 for hackathon demo

### Production (if scaling)
- Railway: $20/month
- Neon: $20/month
- Qdrant: $25/month
- OpenAI: Variable

## Security Checklist

- [ ] All secrets in environment variables (not in code)
- [ ] CORS configured for production domain only
- [ ] Rate limiting enabled (add to backend)
- [ ] HTTPS enabled (automatic on all platforms)
- [ ] Database uses SSL (Neon default)
- [ ] No API keys in frontend code

## Demo Video Requirements

1. **Show live deployment**: Visit production URL
2. **Test authentication**: Signup/login
3. **Test RAG chatbot**: Ask robotics questions
4. **Test agent skills**: Use summarize, quiz, explain
5. **Show personalization**: Demonstrate adaptive content
6. **Show all 22 chapters**: Navigate through modules

## Submission Checklist

- [ ] Frontend deployed and accessible
- [ ] Backend API responding
- [ ] All features working in production
- [ ] Demo video recorded (90 seconds)
- [ ] GitHub repository public
- [ ] README with deployment URLs
- [ ] .env.example provided
- [ ] Tests passing in CI/CD

## Support

If deployment issues:
1. Check GitHub Actions logs
2. Check Railway/Render logs
3. Verify all environment variables
4. Test locally first: `npm run dev` and `uvicorn src.api.main:app --reload`
