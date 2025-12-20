# Deploy Backend to Hugging Face Spaces 🤗

Deploy your FastAPI backend to Hugging Face Spaces for free, reliable hosting that anyone can access!

## Why Hugging Face Spaces?

- ✅ **100% Free** - No credit card required
- ✅ **Always On** - No cold starts or sleep mode
- ✅ **Fast** - Good performance for AI applications
- ✅ **Easy** - Simple deployment process
- ✅ **Popular** - Trusted by ML/AI community
- ✅ **Perfect for Chatbots** - Designed for AI applications

---

## Step-by-Step Deployment Guide

### **Step 1: Create Hugging Face Account**

1. Go to https://huggingface.co/
2. Click **"Sign Up"**
3. Create a free account (use GitHub login for quick setup)
4. Verify your email

---

### **Step 2: Create a New Space**

1. Go to https://huggingface.co/spaces
2. Click **"Create new Space"**
3. Fill in the details:
   - **Space name**: `ai-robotics-chatbot-backend`
   - **License**: `mit`
   - **Select SDK**: Choose **"Docker"** (for FastAPI)
   - **Space hardware**: `CPU basic` (free tier)
   - **Visibility**: `Public` (so anyone can use it)
4. Click **"Create Space"**

---

### **Step 3: Prepare Backend Files**

You need to add a few files to make your backend work on Hugging Face:

#### **3.1: Create Dockerfile**

Create `backend/Dockerfile`:

```dockerfile
FROM python:3.11-slim

WORKDIR /app

# Copy requirements first for better caching
COPY requirements.txt .

# Install dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Expose port 7860 (Hugging Face Spaces default)
EXPOSE 7860

# Run the application
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "7860"]
```

#### **3.2: Create README.md**

Create `backend/README.md`:

```markdown
---
title: AI Robotics Chatbot Backend
emoji: 🤖
colorFrom: green
colorTo: blue
sdk: docker
app_port: 7860
---

# Physical AI & Humanoid Robotics Chatbot Backend

FastAPI backend for the AI-powered chatbot that helps students learn about ROS 2, simulation, NVIDIA Isaac, and VLA models.

## Features

- 🤖 RAG-powered AI chatbot using OpenAI GPT-4
- 🔍 Vector search with Qdrant
- 💾 PostgreSQL database for chat history
- 🔐 JWT authentication
- 🌍 Multi-language support

## API Documentation

Visit `/api/docs` for interactive API documentation.
```

#### **3.3: Create .env.example for Reference**

Create `backend/.env.example`:

```bash
# OpenAI API
OPENAI_API_KEY=your_openai_key_here

# Qdrant Vector Database
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_key_here

# Neon PostgreSQL
DATABASE_URL=your_postgres_url_here

# JWT Secret
JWT_SECRET_KEY=your_jwt_secret_here

# CORS Origins (your frontend URL)
CORS_ORIGINS=["https://shery-ai-robotics-book.vercel.app"]
```

---

### **Step 4: Upload Files to Hugging Face**

#### **Option A: Using Git (Recommended)**

```bash
# Navigate to backend directory
cd backend

# Initialize git if not already done
git init

# Add Hugging Face Space as remote
git remote add hf https://huggingface.co/spaces/YOUR_USERNAME/ai-robotics-chatbot-backend

# Add all files
git add .

# Commit
git commit -m "Initial commit: FastAPI backend"

# Push to Hugging Face
git push hf main
```

#### **Option B: Using Web Interface**

1. Go to your Space page
2. Click **"Files"** tab
3. Click **"Add file"** → **"Upload files"**
4. Upload these files from your `backend/` folder:
   - `main.py`
   - `config.py`
   - `requirements.txt`
   - `Dockerfile`
   - `README.md`
   - All folders: `api/`, `models/`, `database/`, `utils/`
5. Click **"Commit changes to main"**

---

### **Step 5: Configure Environment Variables (Secrets)**

1. In your Space, click **"Settings"** (top right)
2. Scroll to **"Repository secrets"**
3. Click **"New secret"** and add each variable:

```
OPENAI_API_KEY = sk-proj-...  (your actual key)
QDRANT_URL = https://...
QDRANT_API_KEY = eyJhbGci...
DATABASE_URL = postgresql://...
JWT_SECRET_KEY = f08a975...
JWT_ACCESS_TOKEN_EXPIRE_MINUTES = 43200
APP_NAME = Physical AI & Humanoid Robotics Textbook API
APP_VERSION = 1.0.0
DEBUG = false
CORS_ORIGINS = ["https://shery-ai-robotics-book.vercel.app"]
```

**Important**: Copy values from your `backend/.env` file

---

### **Step 6: Wait for Build**

1. Hugging Face will automatically build your Docker container
2. Watch the **"Logs"** tab to see progress
3. Wait 5-10 minutes for the first build
4. Once done, you'll see: **"Running"** status

---

### **Step 7: Get Your Backend URL**

Your backend will be available at:
```
https://YOUR_USERNAME-ai-robotics-chatbot-backend.hf.space
```

Example:
```
https://shery123pk-ai-robotics-chatbot-backend.hf.space
```

Test it:
```
https://YOUR_USERNAME-ai-robotics-chatbot-backend.hf.space/
https://YOUR_USERNAME-ai-robotics-chatbot-backend.hf.space/api/docs
```

---

### **Step 8: Update Frontend to Use Hugging Face Backend**

#### **For Vercel Production:**

1. Go to https://vercel.com/dashboard
2. Select your frontend project: `shery-ai-robotics-book`
3. Go to **Settings** → **Environment Variables**
4. Update `API_URL`:
   ```
   API_URL = https://YOUR_USERNAME-ai-robotics-chatbot-backend.hf.space
   ```
5. Save and **Redeploy**

#### **For Local Development:**

Your local frontend will still use `localhost:8000` automatically. No changes needed!

---

## Testing Your Deployment

### **Test 1: Backend Health Check**
```bash
curl https://YOUR_USERNAME-ai-robotics-chatbot-backend.hf.space/
```
Should return:
```json
{
  "message": "Physical AI & Humanoid Robotics Textbook API",
  "version": "1.0.0",
  "status": "running"
}
```

### **Test 2: API Documentation**
Visit:
```
https://YOUR_USERNAME-ai-robotics-chatbot-backend.hf.space/api/docs
```
Should show FastAPI Swagger UI

### **Test 3: Chatbot**
1. Visit: https://shery-ai-robotics-book.vercel.app/
2. Open chatbot
3. Send a message
4. ✅ **Get AI response!**

---

## Troubleshooting

### Build Failed?

**Check Logs:**
1. Go to your Space
2. Click **"Logs"** tab
3. Look for errors

**Common Issues:**
- Missing dependencies in `requirements.txt`
- Incorrect Dockerfile syntax
- Port mismatch (must be 7860)

### Backend Running but Chatbot Not Working?

**Check CORS:**
1. Make sure `CORS_ORIGINS` includes your frontend URL
2. Update the secret in Hugging Face Settings
3. Restart the Space (Settings → Factory reboot)

**Check Environment Variables:**
1. Verify all secrets are set correctly
2. No typos in secret names
3. Values match your `.env` file

### Backend Times Out?

Hugging Face Spaces can sleep after inactivity on free tier. Upgrade to **persistent** hardware:
1. Settings → Change hardware
2. Select **"CPU basic - Persistent"** (still free!)
3. Click **"Update"**

---

## Advantages Over Other Platforms

| Feature | Hugging Face | Vercel | Railway |
|---------|--------------|--------|---------|
| **Cost** | 100% Free | Limited free | $5/month credit |
| **FastAPI Support** | ⭐⭐⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐⭐⭐ |
| **Always On** | ✅ Yes (persistent) | ❌ Cold starts | ✅ Yes |
| **AI/ML Focus** | ⭐⭐⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐ |
| **Setup Difficulty** | Medium | Complex | Easy |
| **Community** | ML/AI focused | Web apps | Full-stack |

---

## File Structure for Hugging Face

```
backend/
├── Dockerfile              ← NEW (required for HF)
├── README.md               ← NEW (Space description)
├── .env.example           ← NEW (for documentation)
├── main.py
├── config.py
├── requirements.txt
├── api/
│   ├── __init__.py
│   ├── chat.py
│   ├── auth.py
│   └── content.py
├── models/
├── database/
└── utils/
```

---

## Next Steps After Deployment

1. ✅ Test all API endpoints
2. ✅ Verify chatbot works on live site
3. ✅ Monitor Space logs for errors
4. ✅ Share your Space URL with others
5. ✅ Add usage analytics (optional)
6. ✅ Set up persistent hardware if needed

---

## Your Live URLs

After deployment, you'll have:

- **Backend API**: `https://YOUR_USERNAME-ai-robotics-chatbot-backend.hf.space`
- **API Docs**: `https://YOUR_USERNAME-ai-robotics-chatbot-backend.hf.space/api/docs`
- **Frontend**: `https://shery-ai-robotics-book.vercel.app`

---

## Support

- **Hugging Face Docs**: https://huggingface.co/docs/hub/spaces
- **Docker Spaces Guide**: https://huggingface.co/docs/hub/spaces-sdks-docker
- **Secrets Guide**: https://huggingface.co/docs/hub/spaces-overview#managing-secrets

---

**Ready to deploy? Let's make your chatbot live for everyone! 🚀**
