# Chatbot Troubleshooting Guide

## Quick Diagnosis Steps

### 1. Check if Backend is Running

**Windows:**
```powershell
# Check if backend is running on port 8000
netstat -ano | findstr :8000
```

**Linux/Mac:**
```bash
# Check if backend is running on port 8000
lsof -i :8000
# or
netstat -an | grep 8000
```

**Test Backend Manually:**
Open your browser and go to:
- `http://localhost:8000/api/health` - Should return `{"status": "healthy", ...}`
- `http://localhost:8000/api/docs` - Should show API documentation

### 2. Check Browser Console

1. Open your browser's Developer Tools (F12)
2. Go to the **Console** tab
3. Look for error messages when you try to send a chat message
4. Common errors:
   - `Failed to fetch` - Backend not running or CORS issue
   - `NetworkError` - Connection refused
   - `404 Not Found` - Wrong API endpoint
   - `500 Internal Server Error` - Backend error (check backend logs)

### 3. Check Network Tab

1. Open Developer Tools (F12)
2. Go to the **Network** tab
3. Try sending a message
4. Look for the request to `/api/chat/message`
5. Check:
   - **Status Code**: Should be 200 (green) or 429 (rate limit)
   - **Request URL**: Should be `http://localhost:8000/api/chat/message`
   - **Request Payload**: Should have `message`, `session_id`, `user_id`
   - **Response**: Check what the server returned

### 4. Common Issues and Fixes

#### Issue: "Failed to fetch" or "NetworkError"

**Cause:** Backend is not running or not accessible

**Fix:**
1. Make sure backend is running:
   ```bash
   # Windows
   cd backend
   venv\Scripts\activate
   python main.py
   
   # Linux/Mac
   cd backend
   source venv/bin/activate
   python main.py
   ```

2. Verify backend is accessible:
   - Open `http://localhost:8000/api/health` in browser
   - Should see: `{"status": "healthy", ...}`

#### Issue: CORS Error

**Error:** `Access to fetch at 'http://localhost:8000/...' from origin 'http://localhost:3000' has been blocked by CORS policy`

**Fix:**
1. Check `backend/.env` file has correct CORS settings:
   ```bash
   CORS_ORIGINS=["http://localhost:3000","http://localhost:3001"]
   ```

2. Restart the backend after changing `.env`

#### Issue: "500 Internal Server Error"

**Cause:** Backend error (check backend terminal/logs)

**Common Causes:**
- Missing API keys in `backend/.env`
- Qdrant connection failed
- OpenAI API error
- Database connection error

**Fix:**
1. Check backend terminal for error messages
2. Verify all API keys in `backend/.env`:
   - `OPENAI_API_KEY`
   - `QDRANT_URL` and `QDRANT_API_KEY`
   - `DATABASE_URL`
   - `JWT_SECRET_KEY`

#### Issue: "429 Too Many Requests"

**Cause:** Rate limit exceeded (10 requests per minute)

**Fix:** Wait 60 seconds and try again

#### Issue: "404 Not Found"

**Cause:** Wrong API endpoint URL

**Fix:**
1. Check the API URL in browser Network tab
2. Should be: `http://localhost:8000/api/chat/message`
3. Verify backend is running and router is registered

### 5. Verify Backend Configuration

Check `backend/.env` file has all required variables:

```bash
# Required
OPENAI_API_KEY=sk-...
QDRANT_URL=https://...
QDRANT_API_KEY=...
DATABASE_URL=postgresql://...
JWT_SECRET_KEY=...

# Optional (defaults shown)
DEBUG=true
CORS_ORIGINS=["http://localhost:3000"]
```

### 6. Test Backend API Directly

Use curl or Postman to test the API:

```bash
# Windows PowerShell
Invoke-RestMethod -Uri "http://localhost:8000/api/chat/message" `
  -Method POST `
  -ContentType "application/json" `
  -Body '{"message":"What is ROS 2?","session_id":"test-123","user_id":null}'

# Linux/Mac
curl -X POST http://localhost:8000/api/chat/message \
  -H "Content-Type: application/json" \
  -d '{"message":"What is ROS 2?","session_id":"test-123","user_id":null}'
```

Expected response:
```json
{
  "response": "...",
  "citations": [...],
  "session_id": "test-123",
  "timestamp": "..."
}
```

### 7. Check Frontend API Configuration

The frontend uses `getApiUrl()` from `src/utils/apiConfig.ts`:
- Development: Automatically uses `http://localhost:8000`
- Check browser console for the actual URL being called

### 8. Restart Everything

If nothing works, restart both services:

**Windows:**
1. Close all command windows
2. Double-click `START.bat`

**Linux/Mac:**
1. Stop services: `pkill -f "uvicorn\|docusaurus"`
2. Start again: `./START.sh`

## Still Not Working?

1. **Check backend logs** - Look at the terminal running the backend
2. **Check browser console** - Look for JavaScript errors
3. **Check Network tab** - See the actual HTTP request/response
4. **Verify API keys** - Make sure all keys are valid
5. **Test backend directly** - Use curl/Postman to isolate frontend vs backend issues

## Getting Help

If you're still stuck, provide:
1. Error message from browser console
2. HTTP status code from Network tab
3. Backend terminal output/errors
4. Your `backend/.env` file (with keys redacted!)

