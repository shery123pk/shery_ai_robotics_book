# Quick Deploy to Hugging Face 🚀

Deploy your chatbot backend to Hugging Face in 10 minutes and make it live for everyone!

## 🎯 What You'll Do

1. Create a Hugging Face account
2. Create a new Space
3. Upload your backend files
4. Add your API keys as secrets
5. Update frontend to use HF backend
6. ✅ **Your chatbot is live!**

---

## 📝 Quick Steps

### **Step 1: Create HF Account (2 min)**

1. Go to https://huggingface.co/
2. Click "Sign Up" → Sign up with GitHub (easiest)
3. Verify your email

### **Step 2: Create Space (2 min)**

1. Go to https://huggingface.co/spaces
2. Click "Create new Space"
3. Settings:
   - Name: `ai-robotics-chatbot-backend`
   - SDK: **Docker**
   - Hardware: **CPU basic** (free)
   - Visibility: **Public**
4. Click "Create Space"

### **Step 3: Upload Backend Files (3 min)**

**Option A: Use Git (Recommended)**
```bash
cd backend
git remote add hf https://huggingface.co/spaces/YOUR_USERNAME/ai-robotics-chatbot-backend
git add .
git commit -m "Deploy to Hugging Face"
git push hf main
```

**Option B: Web Upload**
1. In your Space, click "Files"
2. Upload these files from `backend/` folder:
   - `Dockerfile`
   - `README.md`
   - `main.py`
   - `config.py`
   - `requirements.txt`
   - All folders: `api/`, `models/`, `database/`, `utils/`

### **Step 4: Add Secrets (2 min)**

1. In your Space, click "Settings"
2. Scroll to "Repository secrets"
3. Click "New secret" for each:

```
Name: OPENAI_API_KEY
Value: (copy from backend/.env)

Name: QDRANT_URL
Value: (copy from backend/.env)

Name: QDRANT_API_KEY
Value: (copy from backend/.env)

Name: DATABASE_URL
Value: (copy from backend/.env)

Name: JWT_SECRET_KEY
Value: (copy from backend/.env)

Name: JWT_ACCESS_TOKEN_EXPIRE_MINUTES
Value: 43200

Name: APP_NAME
Value: Physical AI & Humanoid Robotics Textbook API

Name: APP_VERSION
Value: 1.0.0

Name: DEBUG
Value: false

Name: CORS_ORIGINS
Value: ["https://shery-ai-robotics-book.vercel.app"]
```

### **Step 5: Wait for Build (5 min)**

1. Click "Logs" to watch build progress
2. Wait for "Running" status
3. Your backend URL will be:
   ```
   https://YOUR_USERNAME-ai-robotics-chatbot-backend.hf.space
   ```

### **Step 6: Update Frontend (1 min)**

1. Go to https://vercel.com/dashboard
2. Select: `shery-ai-robotics-book`
3. Settings → Environment Variables
4. Update `API_URL`:
   ```
   https://YOUR_USERNAME-ai-robotics-chatbot-backend.hf.space
   ```
5. Deployments → Redeploy (uncheck cache)

### **Step 7: Test! 🎉**

1. Visit: https://shery-ai-robotics-book.vercel.app/
2. Open chatbot
3. Send a message
4. ✅ **Your chatbot is LIVE!**

---

## 🔗 Your URLs After Deployment

**Backend API:**
```
https://YOUR_USERNAME-ai-robotics-chatbot-backend.hf.space
```

**API Documentation:**
```
https://YOUR_USERNAME-ai-robotics-chatbot-backend.hf.space/api/docs
```

**Frontend:**
```
https://shery-ai-robotics-book.vercel.app
```

---

## 📋 Checklist

Before deploying, make sure you have:

- [ ] Hugging Face account
- [ ] All values from `backend/.env` file
- [ ] Vercel account with frontend deployed
- [ ] Git installed (if using Git method)

---

## 🆘 Need Help?

See the detailed guide: **`HUGGINGFACE_DEPLOYMENT.md`**

Or common issues:
- **Build failed?** Check Logs tab in your Space
- **Can't connect?** Verify CORS_ORIGINS includes frontend URL
- **API errors?** Check all secrets are set correctly

---

## 🎯 Summary

**Time**: ~10 minutes
**Cost**: 100% Free
**Result**: Live chatbot anyone can use!

**Let's deploy! 🚀**
