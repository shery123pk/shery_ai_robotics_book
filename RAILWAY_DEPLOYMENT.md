# Deploy Backend to Railway (Recommended)

Railway is the recommended platform for deploying the FastAPI backend. It has better support for Python/FastAPI than Vercel.

## Why Railway?

- ✅ Native FastAPI support
- ✅ Persistent database connections
- ✅ Auto-deployment from GitHub
- ✅ Free $5/month credit (enough for development)
- ✅ No configuration needed - works out of the box
- ✅ Easy environment variable management

---

## Step-by-Step Deployment

### 1. Sign Up for Railway

1. Go to https://railway.app/
2. Click "Start a New Project"
3. Sign up with GitHub (recommended)

### 2. Deploy from GitHub

1. Click "Deploy from GitHub repo"
2. Select your repository: `shery_ai_robotics_book`
3. Railway will detect it's a Python project

### 3. Configure the Deployment

1. **Root Directory**: Set to `backend`
   - Click on your service → Settings
   - Scroll to "Root Directory"
   - Enter: `backend`
   - Save

2. **Start Command**: Railway should auto-detect, but if needed:
   - Settings → Deploy
   - Start Command: `uvicorn main:app --host 0.0.0.0 --port $PORT`

### 4. Add Environment Variables

Click on your service → Variables tab, then add:

```
OPENAI_API_KEY=<your_openai_key_from_backend/.env>
QDRANT_URL=<your_qdrant_url>
QDRANT_API_KEY=<your_qdrant_api_key>
DATABASE_URL=<your_postgres_url>
JWT_SECRET_KEY=<your_jwt_secret>
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=43200
APP_NAME=Physical AI & Humanoid Robotics Textbook API
APP_VERSION=1.0.0
DEBUG=false
CORS_ORIGINS=["https://shery-ai-robotics-book.vercel.app"]
```

Copy values from your `backend/.env` or `VERCEL_ENV_SETUP.md` file.

### 5. Deploy

1. Click "Deploy"
2. Wait for deployment to complete (~2-3 minutes)
3. **Copy your Railway URL**
   - Example: `https://your-app.up.railway.app`

### 6. Update Frontend

1. Go to Vercel dashboard
2. Select your frontend project: `shery-ai-robotics-book`
3. Settings → Environment Variables
4. Update `API_URL`:
   - Key: `API_URL`
   - Value: `https://your-app.up.railway.app` (your Railway URL)
5. Save

### 7. Redeploy Frontend

1. Deployments tab → Latest deployment
2. "..." menu → Redeploy
3. Uncheck "Use existing Build Cache"
4. Click "Redeploy"

### 8. Test

1. Visit your Railway URL: `https://your-app.up.railway.app/`
   - Should see: `{"message": "Physical AI & Humanoid Robotics Textbook API", ...}`

2. Visit API docs: `https://your-app.up.railway.app/api/docs`
   - Should see interactive FastAPI documentation

3. Test chatbot:
   - Go to: https://shery-ai-robotics-book.vercel.app/
   - Open chatbot → Send message
   - ✅ Should work!

---

## Alternative: Render.com

If Railway doesn't work for you, try Render (also free):

1. Go to https://render.com/
2. "New" → "Web Service"
3. Connect GitHub repository
4. Configure:
   - Name: `shery-ai-robotics-book-backend`
   - Root Directory: `backend`
   - Build Command: `pip install -r requirements.txt`
   - Start Command: `uvicorn main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables (same as above)
6. Deploy

---

## Advantages Over Vercel

| Feature | Railway/Render | Vercel |
|---------|----------------|--------|
| FastAPI Support | ✅ Excellent | ⚠️ Limited |
| Database Connections | ✅ Persistent | ❌ Closes between requests |
| Cold Starts | ⚡ Fast | 🐌 Slow for Python |
| Configuration | 📦 Zero config | 🔧 Complex setup |
| Free Tier | ✅ Generous | ✅ Good but limited |

---

## Troubleshooting

### Deployment Failed?

1. Check build logs in Railway dashboard
2. Verify `requirements.txt` is correct
3. Make sure root directory is set to `backend`

### Backend Not Responding?

1. Check runtime logs in Railway
2. Verify environment variables are set
3. Check that DATABASE_URL and QDRANT_URL are accessible

### Chatbot Still Not Working?

1. Verify `API_URL` in Vercel frontend matches Railway URL
2. Check `CORS_ORIGINS` includes your frontend URL
3. Open browser console (F12) to see error messages

---

## Cost Estimate

**Railway Free Tier:**
- $5 credit per month
- Enough for ~500,000 requests
- No credit card required to start

**Render Free Tier:**
- Free web service
- 750 hours/month
- Spins down after 15 min inactivity

Both are more than enough for development and testing!

---

## Next Steps After Deployment

1. ✅ Test all API endpoints
2. ✅ Verify chatbot works
3. ✅ Test translation feature
4. ✅ Test personalization
5. ✅ Monitor usage in Railway/Render dashboard
6. ✅ Set up custom domain (optional)

---

## Need Help?

- Railway Docs: https://docs.railway.app/
- Render Docs: https://render.com/docs
- FastAPI Deployment: https://fastapi.tiangolo.com/deployment/
