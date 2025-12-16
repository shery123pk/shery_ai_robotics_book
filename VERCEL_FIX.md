# 🔧 Fix Vercel Backend Deployment

Your Vercel is currently serving the **frontend** instead of the **backend API**. Here's how to fix it:

---

## ❌ Problem

When you visit `https://shery-ai-robotics-book.vercel.app/api/health`, you're getting the Docusaurus frontend instead of the FastAPI backend response.

**Root Cause**: Vercel deployed from the project root directory instead of the `backend/` directory.

---

## ✅ Solution: 3 Options

### **Option 1: Delete and Redeploy (Recommended)**

This is the cleanest solution:

#### Step 1: Delete Current Deployment

**Via Vercel Dashboard:**
1. Go to https://vercel.com/dashboard
2. Find project `shery-ai-robotics-book`
3. Click Settings → General → scroll to bottom
4. Click **"Delete Project"**
5. Type the project name to confirm

**Or via CLI:**
```bash
# List your projects
vercel ls

# Remove the project
vercel remove shery-ai-robotics-book
```

#### Step 2: Redeploy from Backend Directory

```bash
# Navigate to backend directory
cd backend

# Login to Vercel (if not already)
vercel login

# Deploy (production)
vercel --prod
```

**Important prompts:**
- **Set up and deploy?** → Yes
- **Which scope?** → Your account
- **Link to existing project?** → No
- **Project name?** → `shery-ai-robotics-backend` (or your choice)
- **In which directory is your code located?** → `./` (current directory - backend)
- **Override settings?** → No

#### Step 3: Set Environment Variables

Go to https://vercel.com/dashboard → Your Project → Settings → Environment Variables

Add these (copy from your `backend/.env` file):

```
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
DATABASE_URL=your_neon_postgres_connection_string
JWT_SECRET_KEY=your_jwt_secret_key
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=43200
CORS_ORIGINS=["https://shery123pk.github.io"]
APP_NAME=Physical AI & Humanoid Robotics Textbook API
DEBUG=false
QDRANT_COLLECTION_NAME=book_embeddings
```

**Where to find these values:**
- All values are in `backend/.env` file
- Copy them exactly as they appear in your `.env` file

#### Step 4: Redeploy with Environment Variables

```bash
# From backend directory
cd backend
vercel --prod
```

#### Step 5: Test the Deployment

```bash
# Replace with your actual Vercel URL
curl https://your-new-backend-url.vercel.app/api/health

# Should return:
# {"status":"healthy","version":"1.0.0"}
```

---

### **Option 2: Update Existing Deployment**

If you want to keep the same URL:

#### Step 1: Update Project Settings

1. Go to https://vercel.com/dashboard
2. Select `shery-ai-robotics-book`
3. Go to **Settings** → **General**
4. Under **Root Directory**, enter: `backend`
5. Click **Save**

#### Step 2: Redeploy

Go to **Deployments** → click the 3 dots on latest deployment → **Redeploy**

---

### **Option 3: Use Git Integration (Auto-Deploy)**

This automatically deploys on every push to GitHub:

#### Step 1: Connect GitHub Repository

1. Go to https://vercel.com/new
2. Click **Import Git Repository**
3. Select `shery123pk/shery_ai_robotics_book`
4. Click **Import**

#### Step 2: Configure Project

- **Project Name**: `shery-ai-robotics-backend`
- **Framework Preset**: Other
- **Root Directory**: `backend` ← **IMPORTANT!**
- **Build Command**: Leave empty
- **Output Directory**: Leave empty
- **Install Command**: `pip install -r requirements.txt`

#### Step 3: Add Environment Variables

Before clicking "Deploy", add all environment variables listed above

#### Step 4: Deploy

Click **Deploy**

#### Step 5: Enable Auto-Deploy

Once deployed:
1. Go to Project Settings → Git
2. Enable "Production Branch": `main`
3. Now every push to `main` auto-deploys backend!

---

## ✅ Verification

After redeployment, test these endpoints:

```bash
# Health check
curl https://your-backend-url.vercel.app/api/health

# Should return:
# {"status":"healthy","version":"1.0.0"}

# Chat health
curl https://your-backend-url.vercel.app/api/chat/health

# Should return:
# {"status":"healthy","qdrant_collections":1,"openai":"connected"}

# API docs (in browser)
https://your-backend-url.vercel.app/api/docs
```

---

## 🔄 After Backend is Fixed

### Update Frontend to Use New Backend URL

1. Edit `.github/workflows/deploy-frontend.yml` line 32:
```yaml
REACT_APP_API_URL: https://your-actual-backend-url.vercel.app
```

2. Commit and push:
```bash
git add .github/workflows/deploy-frontend.yml
git commit -m "Update backend API URL for production"
git push origin main
```

This will trigger GitHub Actions to redeploy frontend with correct backend URL!

---

## 🚫 Common Mistakes to Avoid

1. ❌ **Deploying from project root** → ✅ Deploy from `backend/` directory
2. ❌ **Forgetting environment variables** → ✅ Set them before first request
3. ❌ **Wrong CORS_ORIGINS** → ✅ Must include your GitHub Pages URL
4. ❌ **Not specifying Root Directory** → ✅ Set to `backend` in Vercel settings

---

## 📝 Quick Commands Reference

```bash
# Delete old deployment
cd backend
vercel remove shery-ai-robotics-book

# Fresh deployment
vercel --prod

# Check deployment logs
vercel logs

# List all deployments
vercel ls

# Test deployment
curl https://your-url.vercel.app/api/health
```

---

## 🆘 Still Not Working?

Check these:

1. **Vercel Logs**: Dashboard → Your Project → Deployments → Latest → View Function Logs
2. **Build Logs**: Check if Python dependencies installed correctly
3. **Environment Variables**: Verify all variables are set (no typos)
4. **Requirements**: Make sure `backend/requirements.txt` exists
5. **Python Version**: Vercel uses Python 3.9 by default (our code needs 3.11+)

---

## ✨ Expected Result

After fixing, you should have:

- **Frontend**: `https://shery123pk.github.io/shery_ai_robotics_book/`
- **Backend**: `https://your-backend.vercel.app`
- **API Docs**: `https://your-backend.vercel.app/api/docs`
- **Health Check**: Returns `{"status":"healthy"}`

---

**Need help?** Check Vercel deployment logs for specific error messages.
