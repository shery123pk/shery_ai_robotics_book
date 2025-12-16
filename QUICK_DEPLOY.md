# 🚀 Quick Deployment Guide

**Ready to deploy in 3 steps!**

---

## Step 1: Deploy Backend to Vercel (5 minutes)

```bash
# Install Vercel CLI
npm i -g vercel

# Login
vercel login

# Deploy backend
cd backend
vercel --prod
```

**Set Environment Variables in Vercel Dashboard:**
- `OPENAI_API_KEY` → Your OpenAI API key
- `QDRANT_URL` → Your Qdrant Cloud URL
- `QDRANT_API_KEY` → Your Qdrant API key
- `DATABASE_URL` → Your Neon Postgres URL
- `JWT_SECRET_KEY` → Generate with: `openssl rand -hex 32`
- `CORS_ORIGINS` → `["https://shery123pk.github.io"]`

**Save your backend URL**: `https://your-app.vercel.app`

---

## Step 2: Update Frontend Configuration (2 minutes)

Edit `.github/workflows/deploy-frontend.yml` line 32:
```yaml
REACT_APP_API_URL: https://your-app.vercel.app
```

---

## Step 3: Enable GitHub Pages & Deploy (3 minutes)

1. Go to: https://github.com/shery123pk/shery_ai_robotics_book/settings/pages
2. Under **Source**, select: **GitHub Actions**
3. Push your changes:
   ```bash
   git add .
   git commit -m "Setup deployment"
   git push origin main
   ```

**Your site will be live at:**
`https://shery123pk.github.io/shery_ai_robotics_book/`

---

## ✅ Verify Deployment

### Backend
- Health: https://your-app.vercel.app/api/health
- Docs: https://your-app.vercel.app/api/docs

### Frontend
- Site: https://shery123pk.github.io/shery_ai_robotics_book/
- Test chatbot: Click 💬 button and ask "What is ROS 2?"

---

## 🆘 Troubleshooting

**Chatbot not working?**
- Check backend URL in workflow file
- Verify CORS_ORIGINS includes frontend URL
- Check environment variables in Vercel

**GitHub Actions failing?**
- Check Actions tab for error logs
- Ensure GitHub Pages is enabled
- Verify Node.js version compatibility

---

**Need detailed instructions?** See [DEPLOYMENT.md](./DEPLOYMENT.md)
