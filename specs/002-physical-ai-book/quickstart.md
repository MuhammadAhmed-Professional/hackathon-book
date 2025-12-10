# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

**Feature**: 002-physical-ai-book
**Purpose**: Step-by-step setup instructions for local development and deployment

---

## Prerequisites

Before you begin, ensure you have the following installed and configured:

- **Python 3.11+** ([Download](https://www.python.org/downloads/))
- **Node.js 18+** ([Download](https://nodejs.org/))
- **Git** ([Download](https://git-scm.com/downloads))
- **Code Editor** (VS Code recommended)

**Required Accounts** (All Free Tier):
- **Neon Serverless Postgres**: [Sign up at neon.tech](https://neon.tech)
- **Qdrant Cloud**: [Sign up at cloud.qdrant.io](https://cloud.qdrant.io)
- **OpenAI API**: [Sign up at platform.openai.com](https://platform.openai.com)
- **GitHub**: [Sign up at github.com](https://github.com) (for deployment)

---

## Part 1: Clone Repository and Checkout Branch

```bash
# Clone the repository
git clone https://github.com/<your-username>/<your-repo>.git
cd <your-repo>

# Checkout the Physical AI textbook branch
git checkout 002-physical-ai-book

# Verify you're on the correct branch
git branch
# Should show: * 002-physical-ai-book
```

---

## Part 2: Set Up Neon Serverless Postgres

### Step 1: Create Neon Project

1. Go to [neon.tech](https://neon.tech) and sign in
2. Click **"Create Project"**
3. Choose:
   - **Project Name**: `physical-ai-textbook`
   - **Region**: Select closest to you (e.g., `US East (Ohio)`)
   - **PostgreSQL Version**: Latest (default)
4. Click **"Create Project"**

### Step 2: Get Connection String

1. In your Neon dashboard, go to **"Connection Details"**
2. Copy the connection string (format: `postgresql://user:password@host.region.neon.tech/dbname?sslmode=require`)
3. Save this for Step 4 (environment variables)

### Step 3: Run Database Migration

1. Open the Neon SQL Editor in your browser (from dashboard)
2. Copy the contents of `backend/src/db/migrations/001_initial.sql`
3. Paste into SQL Editor and click **"Run"**
4. Verify tables created:
   ```sql
   SELECT table_name FROM information_schema.tables WHERE table_schema = 'public';
   ```
   You should see: `users`, `conversations`

**Alternative (using psql locally)**:
```bash
# If you have psql installed
psql "<YOUR_NEON_DATABASE_URL>" -f backend/src/db/migrations/001_initial.sql
```

---

## Part 3: Set Up Qdrant Cloud

### Step 1: Create Qdrant Cluster

1. Go to [cloud.qdrant.io](https://cloud.qdrant.io) and sign in
2. Click **"Create Cluster"**
3. Choose:
   - **Cluster Name**: `physical-ai-textbook`
   - **Cloud Provider**: AWS (recommended)
   - **Region**: Select closest to you
   - **Plan**: Free Tier (1GB storage)
4. Click **"Create"**

### Step 2: Get API Credentials

1. Once cluster is created, click on it to open dashboard
2. Go to **"API Keys"** tab
3. Copy:
   - **Cluster URL**: `https://your-cluster.qdrant.io`
   - **API Key**: (click "Show" and copy the key)
4. Save these for Step 4

---

## Part 4: Configure Environment Variables

### Step 1: Create .env File

```bash
# Navigate to backend directory
cd backend

# Copy example environment file
cp .env.example .env

# Open .env in your editor
# Windows: notepad .env
# Mac/Linux: nano .env
```

### Step 2: Fill in Credentials

Edit `.env` and replace placeholders with your actual credentials:

```env
# OpenAI API
OPENAI_API_KEY=sk-proj-your-openai-api-key-here

# Qdrant Cloud
QDRANT_URL=https://your-cluster-id.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here

# Neon Serverless Postgres
NEON_DATABASE_URL=postgresql://user:password@host.region.neon.tech/dbname?sslmode=require

# Better-auth Secret (generate a random 32-byte hex string)
# Windows PowerShell: -join ((0..31) | ForEach-Object { '{0:X2}' -f (Get-Random -Minimum 0 -Maximum 256) })
# Mac/Linux: openssl rand -hex 32
BETTER_AUTH_SECRET=your-random-32-byte-hex-string-here

# Backend API Base URL (for development)
API_BASE_URL=http://localhost:8000
```

**Security Note**: NEVER commit `.env` to Git. It's already in `.gitignore`.

---

## Part 5: Set Up Backend (FastAPI)

### Step 1: Create Virtual Environment

```bash
# Make sure you're in the backend/ directory
cd backend

# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows:
venv\Scripts\activate
# Mac/Linux:
source venv/bin/activate

# Verify activation (should show (venv) in prompt)
```

### Step 2: Install Dependencies

```bash
# Install all Python packages
pip install -r requirements.txt

# Verify installation
pip list | grep fastapi
# Should show: fastapi 0.109.0 (or similar)
```

### Step 3: Start Backend Server

```bash
# Start FastAPI with auto-reload
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000

# You should see:
# INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
# INFO:     Started reloader process
# INFO:     Started server process
# INFO:     Waiting for application startup.
# INFO:     Application startup complete.
```

### Step 4: Test Backend Health Check

Open a new terminal (keep backend running) and test:

```bash
# Test health endpoint
curl http://localhost:8000/ping

# Expected response:
# {"status": "ok", "message": "Physical AI Textbook API is running"}
```

---

## Part 6: Set Up Frontend (Docusaurus)

### Step 1: Install Node Dependencies

```bash
# Open new terminal, navigate to frontend directory
cd frontend

# Install packages
npm install

# Verify installation
npm list docusaurus
# Should show: @docusaurus/core@3.x.x
```

### Step 2: Configure API URL

Edit `frontend/src/config.js`:

```javascript
export const API_BASE_URL = 'http://localhost:8000';
// For production, change to your deployed backend URL
```

### Step 3: Start Development Server

```bash
# Start Docusaurus dev server
npm start

# You should see:
# [INFO] Starting the development server...
# [SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

### Step 4: Test Frontend

1. Open browser to `http://localhost:3000`
2. You should see the Physical AI textbook homepage
3. Navigate to Module 1 → ROS 2 Architecture
4. Verify sidebar navigation works

---

## Part 7: Ingest Textbook Content

With both backend and frontend running, ingest the textbook into Qdrant:

```bash
# In a new terminal
curl -X POST http://localhost:8000/ingest

# Expected response (this may take 30-60 seconds):
# {
#   "status": "success",
#   "chapters_processed": 20,
#   "total_chunks": 850,
#   "collection_name": "physical_ai_textbook"
# }
```

**What this does**:
- Reads all markdown files from `frontend/docs/`
- Chunks each chapter into 200-300 token pieces with 50-token overlap
- Creates embeddings using OpenAI `text-embedding-3-small`
- Stores chunks in Qdrant with metadata (module, chapter, chunk_index)

---

## Part 8: Test RAG Chatbot

### Step 1: Ask a Question via API

```bash
# Test RAG endpoint
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is Physical AI?"}'

# Expected response:
# {
#   "answer_text": "Physical AI refers to AI systems that operate in the physical world...",
#   "sources": [
#     {
#       "module": "Introduction",
#       "chapter": "Introduction to Physical AI",
#       "chunk_id": "chunk_5",
#       "relevance_score": 0.92
#     }
#   ],
#   "conversation_id": 1
# }
```

### Step 2: Test in Browser

1. Go to `http://localhost:3000/module1/ros2-architecture`
2. Scroll to the embedded chatbot at the bottom of the page
3. Type: "How do ROS 2 nodes communicate?"
4. Click **"Send"** or press Enter
5. You should see an answer with source citations

### Step 3: Test Selected-Text Mode

1. On any chapter page, select/highlight a paragraph of text
2. A button should appear: **"Ask about this selection"**
3. Click the button
4. The chatbot opens with the selected text as context
5. Ask a question: "Explain this in simple terms"
6. The answer should be based only on the selected text

---

## Part 9: Test Authentication

### Step 1: Sign Up

1. Go to `http://localhost:3000/signup`
2. Fill in the form:
   - **Email**: `test@example.com`
   - **Password**: `password123`
   - **Software Background**:
     - Programming Languages: Python, C++ (checkboxes or input)
     - Robotics Experience: Intermediate (dropdown)
     - AI/ML Level: Basic (dropdown)
   - **Hardware Background**:
     - RTX GPU Access: Yes (checkbox)
     - RTX GPU Model: RTX 4070 Ti (text input)
     - Jetson Kit: None (dropdown)
     - Robot Hardware: None (dropdown)
3. Click **"Sign Up"**
4. You should be redirected to the textbook homepage with "Logged in as test@example.com" displayed

### Step 2: Verify Database

Check Neon Postgres to verify user was created:

```bash
# Connect to Neon (or use SQL Editor in Neon dashboard)
psql "<YOUR_NEON_DATABASE_URL>"

# Query users table
SELECT id, email, created_at FROM users;

# Expected output:
#  id |       email       |         created_at
# ----+-------------------+----------------------------
#   1 | test@example.com  | 2025-11-27 12:34:56.789
```

### Step 3: Sign In

1. Log out (click profile button → Logout)
2. Go to `http://localhost:3000/signin`
3. Enter email: `test@example.com`, password: `password123`
4. Click **"Sign In"**
5. You should be logged in and see your email in the navbar

### Step 4: Test Authenticated Chatbot

1. While logged in, go to any chapter and ask a question via chatbot
2. Check Neon Postgres to verify conversation was saved with user_id:

```sql
SELECT id, user_id, question, answer, question_type, timestamp
FROM conversations
ORDER BY timestamp DESC
LIMIT 1;

# Expected output:
#  id | user_id |      question      |     answer      | question_type |        timestamp
# ----+---------+--------------------+-----------------+---------------+-------------------------
#   1 |       1 | What is Physical...| Physical AI ... | rag           | 2025-11-27 12:40:15.123
```

---

## Part 10: Build for Production

### Step 1: Build Frontend

```bash
cd frontend

# Build static site
npm run build

# Expected output:
# [SUCCESS] Generated static files in "build".
```

### Step 2: Test Production Build Locally

```bash
# Serve built files
npm run serve

# Open browser to http://localhost:3000
# Test navigation, chatbot, authentication
```

### Step 3: Deploy to GitHub Pages

```bash
# Deploy to GitHub Pages
npm run deploy

# This will:
# 1. Build the site
# 2. Push to gh-pages branch
# 3. GitHub Pages will automatically deploy

# Your site will be live at:
# https://<your-username>.github.io/<your-repo>/
```

### Step 4: Deploy Backend

**Option A: Deploy to Render**
1. Go to [render.com](https://render.com) and sign in
2. Click **"New +" → "Web Service"**
3. Connect your GitHub repository
4. Configure:
   - **Name**: `physical-ai-backend`
   - **Environment**: Python 3.11
   - **Build Command**: `pip install -r requirements.txt`
   - **Start Command**: `uvicorn src.api.main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables (from your `.env` file)
6. Click **"Create Web Service"**
7. Copy the deployed URL (e.g., `https://physical-ai-backend.onrender.com`)

**Option B: Deploy to Railway**
1. Go to [railway.app](https://railway.app) and sign in
2. Click **"New Project" → "Deploy from GitHub repo"**
3. Select your repository and `backend/` directory
4. Add environment variables
5. Deploy and copy the URL

### Step 5: Update Frontend API URL

Edit `frontend/src/config.js` to point to production backend:

```javascript
export const API_BASE_URL = 'https://physical-ai-backend.onrender.com';
```

Rebuild and redeploy frontend:

```bash
npm run build
npm run deploy
```

---

## Troubleshooting

### Issue: "ModuleNotFoundError: No module named 'psycopg2'"
**Solution**: Install psycopg2-binary
```bash
pip install psycopg2-binary
```

### Issue: "Qdrant connection refused"
**Solution**:
- Verify Qdrant URL and API key in `.env`
- Check Qdrant Cloud cluster is running (not paused)
- Test connection: `curl -H "api-key: YOUR_KEY" https://your-cluster.qdrant.io/collections`

### Issue: "OpenAI API rate limit exceeded"
**Solution**:
- Wait a few minutes and retry
- Check your OpenAI usage dashboard for quota limits
- Consider upgrading to paid tier for higher limits

### Issue: "Neon Postgres connection failed: SSL required"
**Solution**:
- Ensure connection string includes `?sslmode=require`
- Example: `postgresql://user:pass@host/db?sslmode=require`

### Issue: "CORS error when calling backend from frontend"
**Solution**:
- Verify backend CORS configuration in `backend/src/api/main.py`
- Ensure `allow_origins=["http://localhost:3000"]` for development
- For production, add your GitHub Pages URL to allowed origins

### Issue: "Frontend build fails with memory error"
**Solution**:
- Increase Node.js memory limit:
  ```bash
  export NODE_OPTIONS="--max-old-space-size=4096"
  npm run build
  ```

### Issue: "Chatbot doesn't appear on pages"
**Solution**:
- Check browser console for errors
- Verify chatbot component is imported in Docusaurus layout
- Ensure backend is running and reachable from frontend

### Issue: "JWT token invalid or expired"
**Solution**:
- Check BETTER_AUTH_SECRET is set correctly in backend `.env`
- Token expires after 24 hours - sign in again
- Clear browser localStorage and cookies, then sign in again

---

## Development Workflow

### Daily Development Routine

1. **Start Backend**:
   ```bash
   cd backend
   source venv/bin/activate  # Windows: venv\Scripts\activate
   uvicorn src.api.main:app --reload
   ```

2. **Start Frontend**:
   ```bash
   cd frontend
   npm start
   ```

3. **Make Changes**:
   - Edit textbook content: `frontend/docs/`
   - Edit backend code: `backend/src/`
   - Both servers auto-reload on file changes

4. **Test Changes**:
   - Test in browser: `http://localhost:3000`
   - Test API: `http://localhost:8000/docs` (Swagger UI)

5. **Commit Changes**:
   ```bash
   git add .
   git commit -m "feat: add NVIDIA Isaac chapter"
   git push origin 002-physical-ai-book
   ```

---

## Next Steps

- **Write Textbook Content**: Edit markdown files in `frontend/docs/module1/`, `module2/`, etc.
- **Customize Styling**: Edit `frontend/src/css/custom.css` for branding
- **Add Diagrams**: Place images in `frontend/static/img/` and reference in markdown
- **Test Agent Skills**: Use `/skills/summarize`, `/skills/quiz`, `/skills/explain` endpoints
- **Create Demo Video**: Record 90-second demo showing all features

---

## Resources

- **Docusaurus Docs**: [docusaurus.io/docs](https://docusaurus.io/docs)
- **FastAPI Docs**: [fastapi.tiangolo.com](https://fastapi.tiangolo.com)
- **Neon Docs**: [neon.tech/docs](https://neon.tech/docs)
- **Qdrant Docs**: [qdrant.tech/documentation](https://qdrant.tech/documentation)
- **OpenAI API Docs**: [platform.openai.com/docs](https://platform.openai.com/docs)
- **Better-auth Docs**: [better-auth.com](https://better-auth.com)

---

## Support

If you encounter issues not covered in troubleshooting:
1. Check GitHub Issues for similar problems
2. Review the spec (`specs/002-physical-ai-book/spec.md`)
3. Review the plan (`specs/002-physical-ai-book/plan.md`)
4. Check the data model (`specs/002-physical-ai-book/data-model.md`)

Good luck with your Physical AI textbook project! 🤖📚
