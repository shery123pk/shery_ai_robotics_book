# Vercel Deployment Guide - Backend & Frontend

This guide will help you deploy both the backend (FastAPI) and frontend (Docusaurus) to Vercel.

## Prerequisites

- Vercel account (sign up at https://vercel.com)
- GitHub repository connected to Vercel
- All environment variables from `backend/.env`

---

## Part 1: Deploy Backend to Vercel

### Option A: Deploy via Vercel Dashboard (Easiest)

1. **Go to Vercel Dashboard**
   - Visit https://vercel.com/dashboard
   - Click "Add New..." → "Project"

2. **Import Your Repository**
   - Select your GitHub repository: `shery_ai_robotics_book`
   - Click "Import"

3. **Configure Backend Project**
   - **Project Name**: `shery-ai-robotics-book-backend`
   - **Framework Preset**: Other
   - **Root Directory**: Click "Edit" → Select `backend` folder
   - **Build Command**: Leave empty
   - **Output Directory**: Leave empty
   - **Install Command**: `pip install -r requirements.txt`

4. **Add Environment Variables**
   Click "Environment Variables" and add these (copy values from your `backend/.env` file):

   ```
   OPENAI_API_KEY=<your_openai_key_from_.env>
   QDRANT_URL=<your_qdrant_url_from_.env>
   QDRANT_API_KEY=<your_qdrant_api_key_from_.env>
   DATABASE_URL=<your_postgres_url_from_.env>
   JWT_SECRET_KEY=<your_jwt_secret_from_.env>
   JWT_ACCESS_TOKEN_EXPIRE_MINUTES=43200
   APP_NAME=Physical AI & Humanoid Robotics Textbook API
   APP_VERSION=1.0.0
   DEBUG=false
   CORS_ORIGINS=["https://shery-ai-robotics-book.vercel.app"]
   ```

   **Important Notes:**
   - Copy the actual values from your `backend/.env` file
   - Make sure to update `CORS_ORIGINS` to include your frontend URL!
   - You can reference `VERCEL_ENV_SETUP.md` (in your local directory, not committed) for the exact values

5. **Deploy**
   - Click "Deploy"
   - Wait for deployment to complete
   - Copy your backend URL (e.g., `https://shery-ai-robotics-book-backend.vercel.app`)

### Option B: Deploy via Vercel CLI

```bash
# Install Vercel CLI globally
npm install -g vercel

# Navigate to backend directory
cd backend

# Login to Vercel
vercel login

# Deploy to Vercel (follow prompts)
vercel

# When prompted:
# - Set up and deploy? Yes
# - Which scope? Select your account
# - Link to existing project? No
# - What's your project name? shery-ai-robotics-book-backend
# - In which directory is your code located? ./ (current directory)

# Deploy to production
vercel --prod

# Copy the production URL shown
```

Then add environment variables in Vercel Dashboard:
- Go to your project → Settings → Environment Variables
- Add all variables listed above

---

## Part 2: Update Frontend to Connect to Backend

### Step 1: Add Backend URL to Frontend Vercel Project

1. **Go to Frontend Project in Vercel**
   - Visit https://vercel.com/dashboard
   - Select your frontend project: `shery-ai-robotics-book`

2. **Add Environment Variable**
   - Go to Settings → Environment Variables
   - Add new variable:
     ```
     Name: API_URL
     Value: https://your-backend-url.vercel.app
     ```
     Replace with your actual backend URL from Part 1

3. **Redeploy Frontend**
   - Go to Deployments
   - Click on the latest deployment
   - Click "..." menu → "Redeploy"
   - Check "Use existing Build Cache" → Redeploy

### Step 2: Verify Deployment

1. **Test Backend**
   - Visit: `https://your-backend-url.vercel.app/`
   - Should see: `{"message": "Physical AI & Humanoid Robotics Textbook API", ...}`

2. **Test Backend API Docs**
   - Visit: `https://your-backend-url.vercel.app/api/docs`
   - Should see FastAPI interactive documentation

3. **Test Frontend Chatbot**
   - Visit: `https://shery-ai-robotics-book.vercel.app/`
   - Open the chatbot (click chat icon in bottom right)
   - Send a test message
   - Should get a response from the AI

---

## Troubleshooting

### HTTP 405 Error?

This has been fixed! The backend now uses Mangum adapter to properly handle FastAPI on Vercel. Make sure you've deployed the latest code.

### Chatbot Still Not Working?

1. **Check Browser Console**
   - Open Developer Tools (F12)
   - Look for CORS errors or API connection errors

2. **Verify CORS Settings**
   - Make sure `CORS_ORIGINS` in backend includes your frontend URL
   - Should be: `["https://shery-ai-robotics-book.vercel.app"]`

3. **Check Environment Variables**
   - In Vercel dashboard, verify all environment variables are set
   - Make sure `API_URL` is set in frontend project

4. **Check Backend Logs**
   - Go to backend project in Vercel
   - Click "Logs" to see runtime errors

5. **Redeploy Backend**
   - If you deployed before the HTTP 405 fix, redeploy your backend
   - Go to Deployments → Latest → "..." → Redeploy

### Backend Deployment Failed?

1. **Check Build Logs**
   - In Vercel dashboard, click on failed deployment
   - Review build logs for errors

2. **Common Issues**
   - Missing dependencies in `requirements.txt`
   - Python version mismatch (should be 3.11)
   - Missing environment variables

---

## Quick Command Reference

```bash
# Deploy backend
cd backend
vercel --prod

# Check frontend deployment status
vercel ls

# View logs
vercel logs <deployment-url>

# Redeploy with environment variables
vercel --prod -e OPENAI_API_KEY=your-key
```

---

## Security Notes

- **NEVER commit `.env` files** to GitHub
- All secrets are managed through Vercel's environment variables
- Use Vercel's environment variable encryption for sensitive data
- Rotate API keys regularly

---

## Next Steps After Deployment

1. **Test all features**:
   - Chat with the AI assistant
   - Check translation feature
   - Test personalization
   - Verify authentication

2. **Monitor usage**:
   - Check Vercel analytics
   - Monitor OpenAI API usage
   - Track Qdrant vector database usage

3. **Set up custom domain** (optional):
   - Add custom domain in Vercel dashboard
   - Update `CORS_ORIGINS` and `API_URL` accordingly

---

## Support

If you encounter issues:
1. Check Vercel deployment logs
2. Review backend API documentation at `/api/docs`
3. Check browser console for frontend errors
4. Verify all environment variables are correctly set
