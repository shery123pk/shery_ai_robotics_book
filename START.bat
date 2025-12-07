@echo off
REM ==============================================================================
REM Physical AI Textbook - One-Click Startup Script (Windows)
REM ==============================================================================
REM This script starts both the frontend (Docusaurus) and backend (FastAPI)
REM Double-click this file to run the complete application
REM ==============================================================================

echo.
echo ========================================
echo  Physical AI Textbook - Starting...
echo ========================================
echo.

REM Check if Node.js is installed
where node >nul 2>nul
if %errorlevel% neq 0 (
    echo [ERROR] Node.js is not installed!
    echo Please install Node.js from https://nodejs.org/
    pause
    exit /b 1
)

REM Check if Python is installed
where python >nul 2>nul
if %errorlevel% neq 0 (
    echo [ERROR] Python is not installed!
    echo Please install Python from https://www.python.org/
    pause
    exit /b 1
)

echo [OK] Node.js detected:
node --version
echo [OK] Python detected:
python --version
echo.

REM Check if backend dependencies are installed
if not exist "backend\venv\" (
    echo [!] Backend virtual environment not found. Creating...
    cd backend
    python -m venv venv
    call venv\Scripts\activate
    echo [*] Installing backend dependencies...
    pip install -r requirements.txt
    cd ..
    echo [OK] Backend setup complete!
    echo.
)

REM Check if frontend dependencies are installed
if not exist "node_modules\" (
    echo [!] Frontend dependencies not found. Installing...
    echo [*] Running npm install (this may take a few minutes)...
    call npm install
    echo [OK] Frontend setup complete!
    echo.
)

REM Check if .env file exists
if not exist "backend\.env" (
    echo [!] WARNING: backend\.env file not found!
    echo [!] Please copy backend\.env.example to backend\.env and add your API keys
    echo [!] The backend will fail to start without proper configuration.
    echo.
    echo Press any key to continue anyway, or Ctrl+C to exit and configure first...
    pause >nul
)

echo ========================================
echo  Starting Services...
echo ========================================
echo.

REM Start backend in a new window
echo [*] Starting Backend (FastAPI) on http://localhost:8000
start "Physical AI Backend" cmd /k "cd backend && venv\Scripts\activate && python main.py"

REM Wait 3 seconds for backend to initialize
timeout /t 3 /nobreak >nul

REM Start frontend in a new window
echo [*] Starting Frontend (Docusaurus) on http://localhost:3000
start "Physical AI Frontend" cmd /k "npm start"

echo.
echo ========================================
echo  SUCCESS! Services Started
echo ========================================
echo.
echo Frontend: http://localhost:3000
echo Backend:  http://localhost:8000
echo API Docs: http://localhost:8000/api/docs
echo.
echo [*] Two command windows have opened:
echo     1. Backend (FastAPI)
echo     2. Frontend (Docusaurus)
echo.
echo [!] To stop the servers, close both command windows
echo     or press Ctrl+C in each window
echo.
echo [*] This window will close in 10 seconds...
timeout /t 10 /nobreak >nul
