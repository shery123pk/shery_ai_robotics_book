#!/bin/bash
# ==============================================================================
# Physical AI Textbook - One-Click Startup Script (Linux/Mac)
# ==============================================================================
# This script starts both the frontend (Docusaurus) and backend (FastAPI)
# Run with: ./START.sh
# Or make executable: chmod +x START.sh && ./START.sh
# ==============================================================================

echo ""
echo "========================================"
echo " Physical AI Textbook - Starting..."
echo "========================================"
echo ""

# Check if Node.js is installed
if ! command -v node &> /dev/null; then
    echo "[ERROR] Node.js is not installed!"
    echo "Please install Node.js from https://nodejs.org/"
    exit 1
fi

# Check if Python is installed
if ! command -v python3 &> /dev/null; then
    echo "[ERROR] Python3 is not installed!"
    echo "Please install Python from https://www.python.org/"
    exit 1
fi

echo "[OK] Node.js detected: $(node --version)"
echo "[OK] Python detected: $(python3 --version)"
echo ""

# Check if backend virtual environment exists
if [ ! -d "backend/venv" ]; then
    echo "[!] Backend virtual environment not found. Creating..."
    cd backend
    python3 -m venv venv
    source venv/bin/activate
    echo "[*] Installing backend dependencies..."
    pip install -r requirements.txt
    cd ..
    echo "[OK] Backend setup complete!"
    echo ""
fi

# Check if frontend dependencies are installed
if [ ! -d "node_modules" ]; then
    echo "[!] Frontend dependencies not found. Installing..."
    echo "[*] Running npm install (this may take a few minutes)..."
    npm install
    echo "[OK] Frontend setup complete!"
    echo ""
fi

# Check if .env file exists
if [ ! -f "backend/.env" ]; then
    echo "[!] WARNING: backend/.env file not found!"
    echo "[!] Please copy backend/.env.example to backend/.env and add your API keys"
    echo "[!] The backend will fail to start without proper configuration."
    echo ""
    read -p "Press Enter to continue anyway, or Ctrl+C to exit and configure first..."
fi

echo "========================================"
echo " Starting Services..."
echo "========================================"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "========================================"
    echo " Shutting down services..."
    echo "========================================"
    kill $BACKEND_PID 2>/dev/null
    kill $FRONTEND_PID 2>/dev/null
    echo "[OK] Services stopped"
    exit 0
}

trap cleanup SIGINT SIGTERM

# Start backend in background
echo "[*] Starting Backend (FastAPI) on http://localhost:8000"
cd backend
source venv/bin/activate
python main.py &
BACKEND_PID=$!
cd ..

# Wait 3 seconds for backend to initialize
sleep 3

# Start frontend in background
echo "[*] Starting Frontend (Docusaurus) on http://localhost:3000"
npm start &
FRONTEND_PID=$!

echo ""
echo "========================================"
echo " SUCCESS! Services Started"
echo "========================================"
echo ""
echo "Frontend: http://localhost:3000"
echo "Backend:  http://localhost:8000"
echo "API Docs: http://localhost:8000/api/docs"
echo ""
echo "[*] Backend PID: $BACKEND_PID"
echo "[*] Frontend PID: $FRONTEND_PID"
echo ""
echo "[!] Press Ctrl+C to stop both servers"
echo ""

# Wait for processes
wait $BACKEND_PID $FRONTEND_PID
