# API Keys & Configuration Guide

## 🔑 Where to Get Your API Keys

This guide will walk you through getting all the API keys needed for the Physical AI & Humanoid Robotics textbook.

---

## 1. OpenAI API Key (Required)

**Cost**: ~$10-50/month depending on usage

### Steps:
1. Go to https://platform.openai.com/signup
2. Sign up or log in
3. Add payment method (required for API access)
4. Go to https://platform.openai.com/api-keys
5. Click "Create new secret key"
6. Copy the key (starts with `sk-...`)
7. **Important**: You can only see it once - save it immediately!

### Add to `.env`:
```bash
OPENAI_API_KEY=sk-proj-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
```

### Usage:
- **GPT-4 Turbo**: Chatbot responses ($0.03/1K tokens)
- **text-embedding-3-small**: Vector embeddings ($0.0001/1K tokens)

---

## 2. Qdrant Cloud API Key (Required)

**Cost**: FREE (up to 1GB)

### Steps:
1. Go to https://cloud.qdrant.io/
2. Sign up with GitHub or Google
3. Click "Create Cluster"
4. Choose **Free Tier** (1GB storage, perfect for this project)
5. Select region (e.g., "US East")
6. Wait ~2 minutes for cluster creation
7. Click on your cluster name
8. Copy the **Cluster URL** (e.g., `https://xxx.us-east-1-0.aws.cloud.qdrant.io:6333`)
9. Go to "API Keys" tab
10. Click "Generate API Key"
11. Copy the API key

### Add to `.env`:
```bash
QDRANT_URL=https://your-cluster-id.us-east-1-0.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key-here
```

### What it's used for:
- Stores vector embeddings of textbook content (~200-300 vectors)
- Enables semantic search for RAG chatbot

---

## 3. Neon Serverless Postgres (Required)

**Cost**: FREE (up to 0.5GB storage, 1 project)

### Steps:
1. Go to https://neon.tech/
2. Sign up with GitHub or Google
3. Click "Create Project"
4. Name it: "ai-robotics-textbook"
5. Select region: "US East (Ohio)" or closest to you
6. Click "Create Project"
7. On the dashboard, click "Connection Details"
8. Copy the **Connection String** (it looks like):
   ```
   postgresql://username:password@ep-xxx-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
   ```

### Add to `.env`:
```bash
DATABASE_URL=postgresql://username:password@ep-xxx-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require
```

### What it's used for:
- Stores user accounts (when authentication is added)
- Stores chat history
- Stores user preferences and personalization data

---

## 4. JWT Secret Key (Required)

**Cost**: FREE

### Steps:
1. Open terminal/command prompt
2. Run this command:
   ```bash
   openssl rand -hex 32
   ```
3. Copy the generated string (64 characters)

### Alternative (if you don't have OpenSSL):
Use this Python script:
```python
import secrets
print(secrets.token_hex(32))
```

### Add to `.env`:
```bash
JWT_SECRET_KEY=your-64-character-random-string-here
```

### What it's used for:
- Signing authentication tokens
- Securing user sessions

---

## 5. CORS Origins (Required)

**Cost**: FREE

### For Local Development:
```bash
CORS_ORIGINS=["http://localhost:3000","http://localhost:3001"]
```

### For Production:
After deploying to GitHub Pages, update with your actual URL:
```bash
CORS_ORIGINS=["https://shery123pk.github.io"]
```

---

## 📝 Complete .env File Setup

### Step 1: Copy the example file
```bash
cd backend
cp .env.example .env
```

### Step 2: Edit `.env` file
Open `backend/.env` in a text editor and fill in all the values:

```bash
# OpenAI
OPENAI_API_KEY=sk-proj-your-actual-key-here

# Qdrant
QDRANT_URL=https://your-cluster.us-east-1-0.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-actual-qdrant-key

# Neon Postgres
DATABASE_URL=postgresql://user:pass@ep-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require

# JWT
JWT_SECRET_KEY=your-64-char-random-string-from-openssl

# App Config
DEBUG=true
CORS_ORIGINS=["http://localhost:3000"]
```

### Step 3: Verify the file
Make sure:
- ✅ No placeholder text like "your-key-here"
- ✅ All URLs are complete
- ✅ No extra spaces or quotes (except in CORS_ORIGINS)

---

## 🚀 Initialize the Database

### Step 1: Install Python dependencies
```bash
cd backend
pip install -r requirements.txt
```

### Step 2: Run database migrations
```bash
python database/run_migrations.py
```

Expected output:
```
✅ Users table created
✅ Chat messages table created
✅ Migrations completed successfully
```

### Step 3: Setup Qdrant collection
```bash
python database/setup_qdrant.py
```

Expected output:
```
✅ Collection 'textbook_content' created
✅ Vector dimension: 1536
✅ Distance metric: Cosine
```

### Step 4: Ingest textbook content
```bash
python ingest_content.py
```

Expected output:
```
Processing: docs/intro.md
Processing: docs/hardware-requirements.md
Processing: docs/module-1/ros2-intro.md
...
✅ Ingested 287 chunks from 18 files
✅ Total vectors in Qdrant: 287
```

This will take ~2-5 minutes depending on your internet speed.

---

## 🧪 Test Your Setup

### Test 1: Backend Health Check
```bash
cd backend
python main.py
```

In another terminal:
```bash
curl http://localhost:8000/api/health
```

Expected output:
```json
{"status":"healthy","version":"1.0.0"}
```

### Test 2: Chatbot API
```bash
curl -X POST http://localhost:8000/api/chat/message \
  -H "Content-Type: application/json" \
  -d '{"message": "What is ROS 2?", "session_id": "test-123"}'
```

Expected output:
```json
{
  "response": "ROS 2 (Robot Operating System 2) is...",
  "citations": [
    {"module": "Module 1", "chapter": "ROS 2 Introduction", "section": "What is ROS 2?"}
  ],
  "session_id": "test-123"
}
```

---

## ⚠️ Security Best Practices

### DO NOT:
- ❌ Commit `.env` file to GitHub
- ❌ Share your API keys publicly
- ❌ Use production keys in development

### DO:
- ✅ Add `.env` to `.gitignore` (already done)
- ✅ Use different keys for dev/staging/production
- ✅ Rotate keys regularly (every 90 days)
- ✅ Monitor API usage in dashboards

---

## 💰 Cost Monitoring

### OpenAI Dashboard
- https://platform.openai.com/usage
- Set up billing alerts (e.g., $10 threshold)

### Qdrant Dashboard
- https://cloud.qdrant.io/
- Monitor storage usage (free tier: 1GB)

### Neon Dashboard
- https://console.neon.tech/
- Monitor database size (free tier: 0.5GB)

---

## 🆘 Troubleshooting

### "Invalid API key" error
- ✅ Check for typos in `.env`
- ✅ Verify key hasn't expired
- ✅ Check API key permissions

### "Connection refused" error
- ✅ Verify Qdrant cluster is running
- ✅ Check Qdrant URL includes port `:6333`
- ✅ Verify Neon project is active

### "Rate limit exceeded" error
- ✅ Wait 60 seconds (10 requests/minute limit)
- ✅ Check OpenAI billing quota

---

## 📚 Additional Resources

- **OpenAI Docs**: https://platform.openai.com/docs
- **Qdrant Docs**: https://qdrant.tech/documentation/
- **Neon Docs**: https://neon.tech/docs/introduction
- **FastAPI Docs**: https://fastapi.tiangolo.com/

---

## ✅ Checklist

Before running the backend, make sure:

- [ ] `.env` file created in `backend/` folder
- [ ] All API keys filled in (no placeholders)
- [ ] Python dependencies installed (`pip install -r requirements.txt`)
- [ ] Database migrations run (`python database/run_migrations.py`)
- [ ] Qdrant collection created (`python database/setup_qdrant.py`)
- [ ] Content ingested (`python ingest_content.py`)
- [ ] Backend starts without errors (`python main.py`)
- [ ] Health check returns `{"status":"healthy"}`
- [ ] Chat API returns responses with citations

---

**Ready?** Run `python backend/main.py` to start your backend! 🚀
