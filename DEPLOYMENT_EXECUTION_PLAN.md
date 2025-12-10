# 🚀 Deployment Execution Plan

**Date**: 2025-11-29
**Goal**: Deploy Physical AI Textbook to production, add translation UI, redeploy, record history

---

## Phase 1: Initial Deployment (NOW)

### Step 1: Verify Everything is Ready ✅

**Backend**:

- [X] 12 tests passing
- [X] Translation API integrated
- [X] Environment variables documented
- [X] Railway/Render configs ready

**Frontend**:

- [X] 20 tests passing
- [X] Docusaurus build works
- [X] GitHub Pages config ready

**Infrastructure**:

- [X] CI/CD workflows created
- [X] Deployment guides written

---

### Step 2: Get API Credentials (10 minutes)

You need to obtain these before deployment:

#### A. OpenAI API Key

```bash
# Visit: https://platform.openai.com/api-keys
# Click "Create new secret key"
# Copy: sk-proj-...
```

#### B. Qdrant Vector Database

```bash
# Visit: https://cloud.qdrant.io
# Sign up (free tier)
# Create cluster: "physical-ai-textbook"
# Copy:
#   - Cluster URL: https://xxx-xxx.qdrant.io:6333
#   - API Key: (from dashboard)
```

#### C. Neon Serverless Postgres

```bash
# Visit: https://neon.tech
# Sign up (free tier)
# Create project: "physical-ai-textbook"
# Copy connection string:
# postgresql://user:password@host.region.neon.tech/dbname?sslmode=require
```

#### D. Generate Auth Secret

```bash
# Windows PowerShell:
[System.Convert]::ToBase64String([System.Security.Cryptography.RandomNumberGenerator]::GetBytes(32))

# OR use online generator:
# https://generate-secret.vercel.app/32
```

**Checklist**:

- [X] OpenAI API key copied
- [X] Qdrant URL + API key copied
- [X] Neon connection string copied
- [X] Auth secret generated (32+ chars)

---

### Step 3: Deploy Backend to Railway (10 minutes)

#### Install Railway CLI

```bash
npm i -g @railway/cli
```

#### Login to Railway

```bash
railway login
# Opens browser for authentication
```

#### Navigate to Backend

```bash
cd backend
```

#### Initialize Railway Project

```bash
railway init
# Select: Create new project
# Name: physical-ai-textbook-backend
```

#### Set Environment Variables

```bash
# Copy these values from Step 2
railway variables set BETTER_AUTH_SECRET="<your-32-char-secret>"
railway variables set OPENAI_API_KEY="sk-proj-..."
railway variables set QDRANT_URL="https://xxx.qdrant.io:6333"
railway variables set QDRANT_API_KEY="<your-qdrant-key>"
railway variables set NEON_DATABASE_URL="postgresql://..."
```

#### Deploy Backend

```bash
railway up
# Wait for build and deployment (~3-5 minutes)
```

#### Get Backend URL

```bash
railway domain
# Output: https://your-app-production.up.railway.app
```

#### Test Backend

```bash
# Replace with your actual Railway URL
curl https://your-app-production.up.railway.app/ping
# Expected: {"status":"ok"}
```

**Checklist**:

- [X] Railway CLI installed
- [X] Railway project created
- [X] Environment variables set
- [X] Backend deployed successfully
- [X] Backend URL obtained: ______________________
- [X] /ping endpoint works

---

### Step 4: Update Frontend Config (2 minutes)

#### Edit Frontend Config

Open `frontend/src/config.js` and update:

```javascript
const config = {
  // Replace with your Railway backend URL
  API_BASE_URL: 'https://your-app-production.up.railway.app'
};

export default config;
```

**Checklist**:

- [X] `frontend/src/config.js` updated with Railway URL
- [X] File saved

---

### Step 5: Deploy Frontend to GitHub Pages (5 minutes)

#### Commit Frontend Config

```bash
git add frontend/src/config.js
git commit -m "Configure production backend URL for Railway"
```

#### Push to Trigger Deployment

```bash
git push origin gh-pages
```

#### Monitor GitHub Actions

```bash
# Visit: https://github.com/YOUR-USERNAME/YOUR-REPO/actions
# Wait for "Deploy to Production" workflow to complete (~2-3 minutes)
```

#### Get Frontend URL

```
https://YOUR-USERNAME.github.io/YOUR-REPO
```

**Checklist**:

- [X] Frontend config committed
- [X] Pushed to gh-pages branch
- [X] GitHub Actions workflow completed
- [X] Frontend URL accessible: ______________________

---

### Step 6: Ingest Documentation (2 minutes)

#### Run Ingestion

```bash
# Replace with your Railway backend URL
curl -X POST https://your-app-production.up.railway.app/ingest
```

This will:

- Read all 22 chapters from the repository
- Generate embeddings with OpenAI (~$0.30)
- Store in Qdrant for RAG retrieval
- Takes ~60-90 seconds

**Expected Output**:

```json
{
  "message": "Documentation ingested successfully",
  "chunks_created": 450,
  "status": "success"
}
```

**Checklist**:

- [X] Ingestion completed successfully
- [X] No errors in response

---

### Step 7: Test Production (10 minutes)

#### Test Backend Endpoints

**Health Check**:

```bash
curl https://your-app.railway.app/ping
```

**Get Skills**:

```bash
curl https://your-app.railway.app/skills
```

**Translation (NEW!)**:

```bash
curl -X POST https://your-app.railway.app/translate \
  -H "Content-Type: application/json" \
  -d '{"content": "Hello robot!", "target_language": "ur"}'
```

#### Test Frontend

**1. Homepage**:

- Visit: https://YOUR-USERNAME.github.io/YOUR-REPO
- Should see: Physical AI & Humanoid Robotics Textbook

**2. Navigation**:

- Click through all 4 modules
- Verify all 22 chapters load

**3. RAG Chatbot**:

- Click floating chatbot icon (bottom-right)
- Ask: "What is ROS 2?"
- Should get answer with source citations

**4. Agent Skills**:

- Navigate to any chapter
- Click "Summarize Section" (if UI implemented)
- Or test via chatbot: "Summarize this chapter"

**5. Authentication**:

- Click "Sign Up" (if visible)
- Create test account
- Verify login works

**Checklist**:

- [ ] Backend health check works
- [ ] Frontend loads correctly
- [ ] All 22 chapters accessible
- [ ] RAG chatbot responds
- [ ] Agent skills work
- [ ] Authentication works (if enabled)

---

## Phase 2: Add Translation UI (After Phase 1)

### Frontend Translation Components

#### 1. Language Selector Component

Create `frontend/src/components/LanguageSelector.tsx`

#### 2. Translation Toggle

Create `frontend/src/components/TranslationToggle.tsx`

#### 3. RTL CSS Support

Add to `frontend/src/css/custom.css`

#### 4. Integration

- Add language selector to navbar
- Add translation toggle to each chapter
- Wire up to `/translate` API

**Estimated Time**: 2 hours

---

## Phase 3: Redeploy with Translation UI

### After Frontend UI is Complete

```bash
# Commit translation UI
git add frontend/src/components/*
git add frontend/src/css/*
git commit -m "Add translation UI with Urdu support"

# Push to redeploy
git push origin gh-pages

# Wait for GitHub Actions (~2-3 minutes)
```

---

## Phase 4: Create History Records

### Deployment History Record

Create `history/prompts/002-physical-ai-book/XXX-deployment-phase1.deployment.prompt.md`

**Contents**:

- Deployment steps executed
- Backend URL
- Frontend URL
- Environment variables configured
- Test results
- Issues encountered (if any)

### Translation Feature Record

Create `history/prompts/002-physical-ai-book/XXX-translation-feature.feature.prompt.md`

**Contents**:

- Translation service implementation
- API endpoints created
- Languages supported
- Code preservation logic
- Glossary terms

---

## Quick Reference

### Your Deployment URLs

**Backend (Railway)**:

- URL: _________________________
- Dashboard: https://railway.app/dashboard

**Frontend (GitHub Pages)**:

- URL: _________________________
- Actions: https://github.com/YOUR-USERNAME/YOUR-REPO/actions

### Environment Variables Needed

```bash
BETTER_AUTH_SECRET=<32-char-secret>
OPENAI_API_KEY=sk-proj-...
QDRANT_URL=https://xxx.qdrant.io:6333
QDRANT_API_KEY=<qdrant-key>
NEON_DATABASE_URL=postgresql://user:pass@host/db?sslmode=require
```

### Cost Tracking

**One-Time Costs**:

- OpenAI ingestion: ~$0.30
- OpenAI translation testing: ~$0.05

**Ongoing Costs** (free tier):

- Railway: Free ($5 credit/month)
- Neon: Free (3GB)
- Qdrant: Free (1GB)
- GitHub Pages: Free

**Total**: ~$0.35 for initial deployment

---

## Troubleshooting

### Backend Won't Start

```bash
# Check logs
railway logs

# Verify environment variables
railway variables

# Common issue: DATABASE_URL should be NEON_DATABASE_URL
railway variables set NEON_DATABASE_URL="postgresql://..."
```

### Frontend Shows CORS Error

```bash
# Check backend CORS configuration
# Add frontend URL to allowed origins in main.py
# Redeploy: railway up
```

### Ingestion Fails

```bash
# Check Qdrant connection
curl -H "api-key: YOUR_KEY" https://your-qdrant-url:6333/collections

# Verify OpenAI key
railway variables | grep OPENAI
```

---

## Next Steps

1. ✅ **Complete Phase 1**: Deploy backend + frontend
2. 🔄 **Phase 2**: Add translation UI components
3. 🔄 **Phase 3**: Redeploy with new UI
4. 🔄 **Phase 4**: Create history records
5. 🎬 **Phase 5**: Record demo video
6. 📝 **Phase 6**: Submit to hackathon

---

**Ready to start Phase 1 deployment!** 🚀

Follow the steps above and update the checklists as you go.
