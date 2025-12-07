# Quick Start Guide - One-Click Startup 🚀

## For Windows Users

### First Time Setup (Do this ONCE)

1. **Double-click `SETUP.bat`**
   - This will install all dependencies
   - Create backend virtual environment
   - Install Python packages
   - Install Node.js packages
   - Set up .env file

2. **Edit `backend\.env`** and add your API keys:
   ```
   OPENAI_API_KEY=sk-your-key-here
   QDRANT_URL=https://your-cluster.qdrant.io
   QDRANT_API_KEY=your-qdrant-key
   DATABASE_URL=postgresql://user:pass@host/db
   JWT_SECRET_KEY=generate-with-openssl-rand-hex-32
   ```

3. **Done!** You're ready to run the app.

---

### Running the Application

**Just double-click `START.bat`** ✨

That's it! The script will:
- ✅ Check if dependencies are installed
- ✅ Start the backend (FastAPI) on http://localhost:8000
- ✅ Start the frontend (Docusaurus) on http://localhost:3000
- ✅ Open two command windows (one for each service)

**To stop the servers:**
- Close both command windows, OR
- Press `Ctrl+C` in each window, OR
- Double-click `STOP.bat` to kill all processes

---

## For Linux/Mac Users

### First Time Setup

1. **Make scripts executable:**
   ```bash
   chmod +x START.sh SETUP.sh
   ```

2. **Run setup:**
   ```bash
   ./SETUP.sh
   ```

3. **Edit `backend/.env`** with your API keys

---

### Running the Application

```bash
./START.sh
```

**To stop the servers:**
- Press `Ctrl+C` in the terminal

---

## What Gets Started?

### Backend (FastAPI)
- **URL**: http://localhost:8000
- **API Docs**: http://localhost:8000/api/docs
- **Services**:
  - RAG Chatbot API
  - Authentication API
  - Translation API (Urdu)
  - Personalization API

### Frontend (Docusaurus)
- **URL**: http://localhost:3000
- **Features**:
  - Interactive textbook (16 chapters)
  - Floating chatbot button
  - Authentication panel
  - Translate to Urdu button
  - Personalize content button

---

## Troubleshooting

### "Node.js is not installed"
- Install from: https://nodejs.org/ (LTS version recommended)

### "Python is not installed"
- Install from: https://www.python.org/ (3.11+ required)

### "Module not found" errors
- Run `SETUP.bat` again to reinstall dependencies

### Backend fails to start
- Check `backend\.env` has all required API keys
- Run migrations: `cd backend && python database\run_migrations.py`

### Frontend fails to start
- Delete `node_modules\` folder
- Run `npm install` again

### Port already in use
- Stop any running instances with `STOP.bat`
- Or change ports in:
  - Backend: `backend/main.py` (line 63: change 8000)
  - Frontend: `package.json` (add `--port 3001` to start script)

---

## Manual Start (If Scripts Don't Work)

### Start Backend Manually:
```bash
cd backend
python -m venv venv
venv\Scripts\activate   # Windows
source venv/bin/activate  # Linux/Mac
pip install -r requirements.txt
python main.py
```

### Start Frontend Manually:
```bash
npm install
npm start
```

---

## File Structure

```
shery_ai_book/
├── START.bat         ← Double-click this (Windows)
├── START.sh          ← Run this (Linux/Mac)
├── SETUP.bat         ← First-time setup (Windows)
├── STOP.bat          ← Stop all services (Windows)
├── backend/          ← FastAPI backend
├── docs/             ← Textbook content
├── src/              ← React components
└── node_modules/     ← Frontend dependencies
```

---

## Next Steps

After starting the app:

1. **Visit http://localhost:3000**
2. **Sign up** for an account (test the authentication)
3. **Open a chapter** (e.g., Module 1 > ROS 2 Introduction)
4. **Try the chatbot** - Click the chat button (bottom-right)
5. **Try personalization** - Click "✨ Personalize for You"
6. **Try translation** - Click "🇵🇰 Translate to Urdu"

---

## Production Deployment

Once you've tested locally, deploy to production:

### Frontend → GitHub Pages:
```bash
npm run build
GIT_USER=your-username npm run deploy
```

### Backend → Vercel:
```bash
npm install -g vercel
vercel --prod
```

See `DEPLOYMENT.md` for detailed instructions.

---

**Happy coding! 🚀**
