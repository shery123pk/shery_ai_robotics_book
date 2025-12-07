# ⚡ Quick Start: Where to Put API Keys

## 📍 EXACT FILE LOCATION: `backend/.env`

All your API keys go in ONE file: **`backend/.env`**

---

## 🚀 Quick Setup (3 Steps)

### Step 1: Create the `.env` File

```bash
cd backend
cp .env.example .env
```

Or manually create `backend/.env` file

### Step 2: Open `backend/.env` in a Text Editor

Use VS Code, Notepad++, or any text editor:
```bash
code backend/.env
```

### Step 3: Fill in Your API Keys

Copy this template and paste into `backend/.env`:

```bash
# ==============================================
# PASTE YOUR API KEYS HERE
# ==============================================

# 1. OpenAI API Key
# Get it from: https://platform.openai.com/api-keys
OPENAI_API_KEY=sk-proj-xxxxx-YOUR-ACTUAL-KEY-HERE-xxxxx

# 2. Qdrant Cloud
# Get it from: https://cloud.qdrant.io/
QDRANT_URL=https://your-cluster-id.us-east-1-0.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key-here

# 3. Neon Postgres
# Get it from: https://console.neon.tech/
DATABASE_URL=postgresql://username:password@ep-xxx-xxx.us-east-2.aws.neon.tech/neondb?sslmode=require

# 4. JWT Secret (generate with: openssl rand -hex 32)
JWT_SECRET_KEY=your-64-character-random-string-here

# 5. App Configuration
DEBUG=true
CORS_ORIGINS=["http://localhost:3000"]
```

---

## 🔑 Where to Get Each API Key

### 1. OpenAI API Key ⭐ REQUIRED

**Website**: https://platform.openai.com/api-keys

**Steps**:
1. Click "Sign Up" → Create account
2. Add payment method (credit card required)
3. Click "Create new secret key"
4. Copy the key (starts with `sk-proj-` or `sk-`)
5. ⚠️ **Save it immediately - you can only see it once!**

**Paste in `.env`**:
```bash
OPENAI_API_KEY=sk-proj-abc123xyz789...
```

**Cost**: ~$5-20/month (pay-as-you-go)

---

### 2. Qdrant Cloud API Key ⭐ REQUIRED

**Website**: https://cloud.qdrant.io/

**Steps**:
1. Sign up with GitHub or Google
2. Click "Create Cluster"
3. Choose **Free Tier** (1GB - perfect for this project!)
4. Select region: "US East"
5. Wait 2 minutes for cluster creation
6. Click on your cluster name
7. **Copy the Cluster URL** (looks like):
   ```
   https://abc123xyz.us-east-1-0.aws.cloud.qdrant.io:6333
   ```
8. Go to "API Keys" tab → "Generate API Key"
9. Copy the API key

**Paste in `.env`**:
```bash
QDRANT_URL=https://abc123xyz.us-east-1-0.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-key-here
```

**Cost**: FREE (1GB limit)

---

### 3. Neon Postgres Database ⭐ REQUIRED

**Website**: https://neon.tech/

**Steps**:
1. Sign up with GitHub or Google
2. Click "Create Project"
3. Name it: "ai-robotics-textbook"
4. Select region: "US East (Ohio)"
5. Click "Create Project"
6. On dashboard, click "Connection Details"
7. **Copy the entire connection string** (looks like):
   ```
   postgresql://alex:AbCdEf123@ep-cool-darkness-123456.us-east-2.aws.neon.tech/neondb?sslmode=require
   ```

**Paste in `.env`**:
```bash
DATABASE_URL=postgresql://alex:AbCdEf123@ep-cool-darkness-123456.us-east-2.aws.neon.tech/neondb?sslmode=require
```

**Cost**: FREE (0.5GB limit, 1 project)

---

### 4. JWT Secret Key ⭐ REQUIRED

**Generate it yourself** (FREE)

**Option A - Using Terminal**:
```bash
openssl rand -hex 32
```

**Option B - Using Python**:
```python
import secrets
print(secrets.token_hex(32))
```

**Option C - Using Online Generator**:
Go to: https://generate-secret.vercel.app/32

Copy the generated 64-character string.

**Paste in `.env`**:
```bash
JWT_SECRET_KEY=a1b2c3d4e5f6g7h8...
```

---

## ✅ Complete `.env` File Example

Here's what your `backend/.env` should look like when filled in:

```bash
# OpenAI
OPENAI_API_KEY=sk-proj-v1_abc123xyz789...

# Qdrant
QDRANT_URL=https://abc123.us-east-1-0.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=qdr_abc123xyz789...

# Neon Postgres
DATABASE_URL=postgresql://myuser:mypass123@ep-cool-123.us-east-2.aws.neon.tech/neondb?sslmode=require

# JWT
JWT_SECRET_KEY=a1b2c3d4e5f6g7h8i9j0k1l2m3n4o5p6...

# App
DEBUG=true
CORS_ORIGINS=["http://localhost:3000"]
APP_NAME="Physical AI & Humanoid Robotics Textbook API"
APP_VERSION=1.0.0
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=43200
```

---

## 🧪 Test Your Setup

### 1. Install Dependencies
```bash
cd backend
pip install -r requirements.txt
```

### 2. Run Database Migrations
```bash
python database/run_migrations.py
```

Expected output:
```
✅ Users table created
✅ Chat messages table created
✅ Migrations completed
```

### 3. Setup Qdrant
```bash
python database/setup_qdrant.py
```

Expected output:
```
✅ Collection 'textbook_content' created
```

### 4. Ingest Content
```bash
python ingest_content.py
```

Expected output:
```
Processing: docs/intro.md
Processing: docs/module-1/ros2-intro.md
...
✅ Ingested 287 chunks from 18 files
```

### 5. Start Backend
```bash
python main.py
```

Expected output:
```
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

### 6. Test API
Open browser: http://localhost:8000/api/docs

You should see the API documentation!

---

## ⚠️ Common Mistakes

### ❌ DON'T:
- Put `.env` file in the wrong folder (it must be in `backend/` folder!)
- Commit `.env` to GitHub (it's in .gitignore for a reason!)
- Share your API keys publicly
- Use production keys in development

### ✅ DO:
- Keep `.env` file in `backend/` folder
- Use different keys for dev/staging/production
- Monitor your API usage in dashboards
- Rotate keys every 90 days

---

## 🆘 Troubleshooting

### "File not found: .env"
Your `.env` file is not in the right place!

**Fix**:
```bash
# Check current location
pwd  # Should show: .../backend

# Create .env here
cp .env.example .env
```

### "Invalid API key" error
**Fix**:
1. Open `backend/.env`
2. Check for typos
3. Make sure you copied the FULL key (no spaces, no line breaks)
4. Check the key is still valid in the dashboard

### "Connection refused" error
**Fix**:
1. Verify your Qdrant cluster is running (check dashboard)
2. Check Qdrant URL includes `:6333` port
3. Verify Neon project is active

### " ModuleNotFoundError: No module named 'passlib'"
**Fix**:
```bash
cd backend
pip install -r requirements.txt
```

---

## 📊 Cost Summary

| Service | Free Tier | Expected Monthly Cost |
|---------|-----------|----------------------|
| **OpenAI** | $0 (first $5 free) | $10-50 |
| **Qdrant** | 1GB free | $0 |
| **Neon** | 0.5GB free | $0 |
| **JWT** | Unlimited | $0 |
| **Total** | | **$10-50/month** |

---

## 🎯 Quick Reference

```
PROJECT_ROOT/
├── shery_ai_book/
│   ├── backend/
│   │   ├── .env          👈 PUT API KEYS HERE!
│   │   ├── .env.example  (template)
│   │   ├── main.py
│   │   ├── config.py
│   │   └── ...
│   ├── docs/
│   ├── src/
│   └── ...
```

---

## ✅ Final Checklist

Before running backend:

- [ ] Created `backend/.env` file
- [ ] Copied API keys from dashboards
- [ ] OpenAI API key filled in (`sk-...`)
- [ ] Qdrant URL filled in (`https://...`)
- [ ] Qdrant API key filled in
- [ ] Neon DATABASE_URL filled in (`postgresql://...`)
- [ ] JWT_SECRET_KEY generated (64 characters)
- [ ] No placeholder text left (like "your-key-here")
- [ ] Ran `pip install -r requirements.txt`
- [ ] Ran `python database/run_migrations.py`
- [ ] Ran `python database/setup_qdrant.py`
- [ ] Ran `python ingest_content.py`
- [ ] Backend starts: `python main.py`
- [ ] API docs work: http://localhost:8000/api/docs

---

**Done?** You're ready to run the backend! 🚀

Run:
```bash
cd backend
python main.py
```

Then visit: **http://localhost:8000/api/docs**
