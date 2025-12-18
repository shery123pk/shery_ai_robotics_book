# Quick Fix Guide - Connect Frontend to Backend

## Problem
You removed the backend API URL from Vercel frontend, so the chatbot can't connect to the backend.

## Solution (3 Steps - Takes 2 Minutes)

---

### Step 1: Find Your Backend URL

First, we need to know where your backend is deployed.

**If you deployed backend to Vercel:**
1. Go to https://vercel.com/dashboard
2. Find your **backend** project (probably named like `shery-ai-robotics-book-backend`)
3. Click on it
4. Copy the **Domain** URL (e.g., `https://shery-ai-robotics-book-backend.vercel.app`)

**If you haven't deployed the backend yet:**
- Option A: Deploy to Railway (5 minutes) - See `RAILWAY_DEPLOYMENT.md`
- Option B: Deploy to Vercel backend - See `DEPLOYMENT_CHECKLIST.md`

---

### Step 2: Add API_URL to Frontend Vercel Project

1. Go to https://vercel.com/dashboard
2. Select your **FRONTEND** project: `shery-ai-robotics-book` (not the backend!)
3. Go to **Settings** → **Environment Variables**
4. Click **"Add New"** or **"Edit"** if it exists:
   - **Key**: `API_URL`
   - **Value**: Your backend URL from Step 1
   - **Example**: `https://shery-ai-robotics-book-backend.vercel.app`
   - **Environments**: Select all three (Production, Preview, Development)
5. Click **"Save"**

---

### Step 3: Redeploy Frontend

1. Stay in your frontend project in Vercel
2. Go to **Deployments** tab
3. Click on the **latest deployment** (top of the list)
4. Click the **"..."** menu (three dots)
5. Click **"Redeploy"**
6. **IMPORTANT**: **Uncheck** "Use existing Build Cache"
7. Click **"Redeploy"**
8. Wait 1-2 minutes for deployment to complete

---

## Step 4: Test the Chatbot

1. Visit: https://shery-ai-robotics-book.vercel.app/
2. Click the **chatbot icon** (bottom right corner)
3. Send a test message: "What is ROS 2?"
4. ✅ **Should get AI response!**

---

## Troubleshooting

### Chatbot Still Doesn't Work?

**Check 1: Verify API_URL is Set**
- Vercel Frontend → Settings → Environment Variables
- Make sure `API_URL` is there with your backend URL

**Check 2: Verify Backend is Running**
- Visit your backend URL directly
- Should see: `{"message": "Physical AI & Humanoid Robotics Textbook API", ...}`
- If you get 500 error, backend needs fixing

**Check 3: Check Browser Console**
- Open Developer Tools (F12)
- Go to Console tab
- Look for errors when you send a chat message
- Share the error message with me

**Check 4: Check CORS**
- Make sure backend has your frontend URL in `CORS_ORIGINS`
- Should be: `["https://shery-ai-robotics-book.vercel.app"]`

---

## What Each URL Does

**Frontend URL**: `https://shery-ai-robotics-book.vercel.app`
- This is your website (Docusaurus)
- Users visit this URL
- The chatbot UI is here

**Backend URL**: `https://your-backend.vercel.app` (or Railway URL)
- This is your API (FastAPI)
- The chatbot sends messages to this URL
- Handles AI responses, database, etc.

**API_URL Environment Variable**:
- Tells the frontend WHERE to find the backend
- Without it, frontend doesn't know where to send chat messages
- Must be set in Vercel frontend project

---

## Quick Reference

```
Frontend Project (Vercel):
  - Name: shery-ai-robotics-book
  - URL: https://shery-ai-robotics-book.vercel.app
  - Env Var Needed: API_URL = <your-backend-url>

Backend Project (Vercel or Railway):
  - Name: shery-ai-robotics-book-backend
  - URL: https://your-backend.vercel.app (or Railway URL)
  - Must have: CORS_ORIGINS with frontend URL
```

---

## What's Your Backend URL?

Please tell me:
1. Did you deploy the backend to Vercel or Railway?
2. What is your backend URL?

Then I'll help you add it to the frontend!
