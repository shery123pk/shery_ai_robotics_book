# Deployment Guide

Complete guide for deploying the Physical AI & Humanoid Robotics interactive textbook.

## Overview

This project consists of:
- **Frontend**: Docusaurus 3.x static site (React/TypeScript)
- **Backend**: FastAPI Python server (RAG chatbot)
- **Database**: Qdrant (vector DB) + Neon Postgres (SQL)
- **AI**: OpenAI GPT-4 + Embeddings

## Prerequisites

- Node.js 18+ and npm
- Python 3.11+
- OpenAI API key
- Qdrant Cloud account (or local Qdrant)
- Neon Serverless Postgres database

## Environment Variables

Create `.env` file in the `backend/` directory:

```bash
# OpenAI
OPENAI_API_KEY=sk-your-openai-api-key

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# Neon Postgres
DATABASE_URL=postgresql://user:password@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require

# JWT Secret (generate with: openssl rand -hex 32)
JWT_SECRET_KEY=your-secret-key-here

# Backend Config
DEBUG=false
CORS_ORIGINS=["https://yourdomain.github.io"]
```

## Installation

### 1. Frontend Setup

```bash
cd shery_ai_book
npm install
npm run build
```

### 2. Backend Setup

```bash
cd backend
pip install -r requirements.txt
```

### 3. Database Setup

```bash
# Run SQL migrations
python database/run_migrations.py

# Setup Qdrant collection
python database/setup_qdrant.py
```

### 4. Content Ingestion

Ingest textbook content into Qdrant:

```bash
cd backend
python ingest_content.py
```

This will:
- Parse all markdown files in `docs/`
- Chunk content into ~800 char sections
- Generate embeddings with OpenAI
- Upload to Qdrant vector database

**Expected output**: ~200-300 chunks from 16 chapters

## Running Locally

### Development Mode

**Terminal 1 - Frontend**:
```bash
cd shery_ai_book
npm start
```
Opens http://localhost:3000

**Terminal 2 - Backend**:
```bash
cd backend
python main.py
```
Runs on http://localhost:8000

**Terminal 3 - Test Chatbot**:
```bash
curl -X POST http://localhost:8000/api/chat/message \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?", "session_id": "test-123"}'
```

## Production Deployment

### Frontend (GitHub Pages)

1. **Update `docusaurus.config.ts`**:
```typescript
{
  url: 'https://YOUR_USERNAME.github.io',
  baseUrl: '/YOUR_REPO_NAME/',
  organizationName: 'YOUR_USERNAME',
  projectName: 'YOUR_REPO_NAME',
}
```

2. **Build and Deploy**:
```bash
npm run build
GIT_USER=YOUR_USERNAME npm run deploy
```

### Backend (Vercel / Railway)

#### Option A: Vercel

1. Create `vercel.json`:
```json
{
  "builds": [
    {
      "src": "backend/main.py",
      "use": "@vercel/python"
    }
  ],
  "routes": [
    {
      "src": "/api/(.*)",
      "dest": "backend/main.py"
    }
  ]
}
```

2. Deploy:
```bash
vercel --prod
```

#### Option B: Railway

1. Create `railway.toml`:
```toml
[build]
builder = "NIXPACKS"

[deploy]
startCommand = "cd backend && uvicorn main:app --host 0.0.0.0 --port $PORT"
```

2. Deploy:
```bash
railway up
```

### Update Frontend API URL

In `src/components/ChatBot/ChatBot.tsx`:
```typescript
const response = await fetch('https://your-backend.vercel.app/api/chat/message', {
  ...
});
```

## Verification Checklist

### Frontend
- [ ] Site builds without errors (`npm run build`)
- [ ] All navigation links work
- [ ] All 16 chapters render correctly
- [ ] Mobile responsive

### Backend
- [ ] API health check: `GET /api/health`
- [ ] Chat endpoint works: `POST /api/chat/message`
- [ ] Qdrant contains vectors: Check collection stats
- [ ] Database migrations applied

### Chatbot
- [ ] Chatbot icon appears on all pages
- [ ] Can send messages
- [ ] Receives AI responses
- [ ] Citations appear
- [ ] Rate limiting works (10 msgs/min)

## Troubleshooting

### "Module 'api' not found"
```bash
cd backend
touch api/__init__.py
```

### "Qdrant collection not found"
```bash
python database/setup_qdrant.py
python ingest_content.py
```

### CORS errors
Update `backend/config.py`:
```python
cors_origins = ["https://yourdomain.github.io", "http://localhost:3000"]
```

### Slow responses
- Check OpenAI API quota
- Reduce Qdrant search limit (5 → 3)
- Enable response caching

## Performance Optimization

1. **Frontend**:
   - Enable code splitting
   - Lazy load images
   - Use Docusaurus production build

2. **Backend**:
   - Cache embeddings
   - Use connection pooling
   - Enable API rate limiting

3. **Database**:
   - Index frequently queried fields
   - Use Qdrant disk storage for large collections

## Monitoring

### Backend Logs
```bash
tail -f backend/logs/app.log
```

### Qdrant Stats
```python
from qdrant_client import QdrantClient
client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
info = client.get_collection("textbook_content")
print(f"Points: {info.points_count}")
```

### Database
```sql
SELECT COUNT(*) FROM chat_messages;
SELECT COUNT(*) FROM users;
```

## Cost Estimation

### Monthly costs (estimated):

- **OpenAI API**: ~$10-50 (depends on usage)
  - Embeddings: $0.0001/1K tokens
  - GPT-4: $0.03/1K tokens

- **Qdrant Cloud**: Free (up to 1GB)

- **Neon Postgres**: Free (up to 0.5GB)

- **Vercel/Railway**: Free tier available

**Total**: $10-50/month for moderate usage

## Security Checklist

- [ ] API keys in `.env`, not in code
- [ ] CORS restricted to your domain
- [ ] Rate limiting enabled
- [ ] SQL injection prevention (asyncpg)
- [ ] Input validation on all endpoints
- [ ] HTTPS enabled in production

## Support

For issues:
1. Check logs (`backend/logs/`)
2. Verify environment variables
3. Test with curl/Postman
4. Review Qdrant/Neon dashboards

## Next Steps

After deployment:
1. Test chatbot with 10+ questions
2. Verify all citations work
3. Check mobile responsiveness
4. Set up monitoring/analytics
5. Create demo video

---

**Deployed successfully?** Share your link and celebrate! 🎉
