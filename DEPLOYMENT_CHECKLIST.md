# Backend + Frontend Deployment Checklist

**IMPORTANT:** The backend should be deployed to **Railway** (not Vercel) for best results. Vercel has issues with FastAPI apps.

## ⚠️ Deployment Platform Decision

- **Frontend**: ✅ Vercel (already working)
- **Backend**: 🚂 **Railway** (recommended) or Render

See `DEPLOYMENT_OPTIONS.md` for why Railway is better than Vercel for the backend.

---

## Quick Start (Railway - 5 minutes)

Follow these steps in order:

## ✅ Step 1: Deploy Backend to Railway

1. [ ] Go to https://railway.app/
2. [ ] Sign up with GitHub
3. [ ] Click "Start a New Project"
4. [ ] Select "Deploy from GitHub repo"
5. [ ] Choose repository: `shery_ai_robotics_book`
6. [ ] Configure deployment:
   - [ ] Settings → Root Directory → Enter `backend` → Save
   - [ ] (Railway auto-detects the start command)
7. [ ] Add environment variables (Variables tab):
   - [ ] OPENAI_API_KEY (from `backend/.env`)
   - [ ] QDRANT_URL
   - [ ] QDRANT_API_KEY
   - [ ] DATABASE_URL
   - [ ] JWT_SECRET_KEY
   - [ ] JWT_ACCESS_TOKEN_EXPIRE_MINUTES=43200
   - [ ] APP_NAME=Physical AI & Humanoid Robotics Textbook API
   - [ ] APP_VERSION=1.0.0
   - [ ] DEBUG=false
   - [ ] CORS_ORIGINS=["https://shery-ai-robotics-book.vercel.app"]
8. [ ] Click "Deploy" (if not auto-deployed)
9. [ ] Wait for deployment (~2-3 minutes)
10. [ ] **Copy your Railway URL** (Settings → Domain)
   - Example: `https://your-app.up.railway.app`

### Test Backend:
11. [ ] Visit: `https://your-railway-url.up.railway.app/`
   - Should see: `{"message": "Physical AI & Humanoid Robotics Textbook API", ...}`
12. [ ] Visit: `https://your-railway-url.up.railway.app/api/docs`
   - Should see: FastAPI interactive documentation

---

## ✅ Step 2: Update Frontend Environment Variable

1. [ ] Go to https://vercel.com/dashboard
2. [ ] Select your frontend project: `shery-ai-robotics-book`
3. [ ] Go to **Settings** → **Environment Variables**
4. [ ] Click "Add New"
   - Key: `API_URL`
   - Value: Your backend URL (from Step 1, step 8)
   - Example: `https://shery-ai-robotics-book-backend.vercel.app`
   - Select all environments (Production, Preview, Development)
5. [ ] Click "Save"

---

## ✅ Step 3: Redeploy Frontend

1. [ ] In Vercel frontend project, go to **Deployments** tab
2. [ ] Click on the latest deployment
3. [ ] Click "..." menu → **Redeploy**
4. [ ] **Uncheck** "Use existing Build Cache"
5. [ ] Click "Redeploy"
6. [ ] Wait for deployment to complete

---

## ✅ Step 4: Test Everything

1. [ ] Visit: https://shery-ai-robotics-book.vercel.app/
2. [ ] Verify no error banner appears
3. [ ] Click the chatbot icon (bottom right corner)
4. [ ] Send a test message: "What is ROS 2?"
5. [ ] Wait for AI response
6. [ ] ✅ **SUCCESS!** Your chatbot is working!

---

## 🔍 Troubleshooting

If chatbot doesn't work:

### Check Browser Console (F12):
- [ ] Look for CORS errors
- [ ] Look for API connection errors
- [ ] Note any error messages

### Verify Backend:
- [ ] Backend URL is accessible
- [ ] `/api/docs` endpoint works
- [ ] `CORS_ORIGINS` includes frontend URL

### Verify Frontend:
- [ ] `API_URL` environment variable is set correctly
- [ ] Frontend was redeployed after adding `API_URL`
- [ ] Build cache was cleared during redeploy

### Check Logs:
- [ ] Backend logs in Vercel (Backend project → Logs)
- [ ] Frontend logs in Vercel (Frontend project → Logs)

---

## 📝 Reference Files

- **Detailed Guide**: See `VERCEL_DEPLOYMENT_GUIDE.md`
- **Environment Variables**: See `VERCEL_ENV_SETUP.md` (local only, contains your actual credentials)
- **Backend .env**: See `backend/.env` (contains all your API keys)

---

## 🎉 After Successful Deployment

Once everything is working:

- [ ] Test all chatbot features
- [ ] Try translation feature
- [ ] Test personalization
- [ ] Monitor API usage on OpenAI dashboard
- [ ] Set up monitoring/alerting (optional)

---

## 📞 Need Help?

If you get stuck:
1. Check the detailed guide: `VERCEL_DEPLOYMENT_GUIDE.md`
2. Review Vercel deployment logs
3. Check browser console for errors
4. Verify all environment variables are correctly set
