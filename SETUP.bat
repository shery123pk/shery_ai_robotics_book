@echo off
REM ==============================================================================
REM Physical AI Textbook - First Time Setup (Windows)
REM ==============================================================================
REM Run this script ONCE before using START.bat for the first time
REM This will install all dependencies and configure the environment
REM ==============================================================================

echo.
echo ========================================
echo  Physical AI Textbook - First Setup
echo ========================================
echo.

REM Check Node.js
where node >nul 2>nul
if %errorlevel% neq 0 (
    echo [ERROR] Node.js is not installed!
    echo Please install Node.js from https://nodejs.org/
    pause
    exit /b 1
)

REM Check Python
where python >nul 2>nul
if %errorlevel% neq 0 (
    echo [ERROR] Python is not installed!
    echo Please install Python from https://www.python.org/
    pause
    exit /b 1
)

echo [OK] Node.js:
node --version
echo [OK] Python:
python --version
echo.

REM Step 1: Create backend virtual environment
echo ========================================
echo  Step 1/4: Setting up Backend
echo ========================================
echo.
cd backend

if exist "venv\" (
    echo [!] Virtual environment already exists, skipping...
) else (
    echo [*] Creating virtual environment...
    python -m venv venv
    echo [OK] Virtual environment created
)

echo [*] Activating virtual environment...
call venv\Scripts\activate

echo [*] Installing backend dependencies...
pip install -r requirements.txt
echo [OK] Backend dependencies installed
echo.

REM Step 2: Setup environment variables
echo ========================================
echo  Step 2/4: Environment Configuration
echo ========================================
echo.

if exist ".env" (
    echo [!] .env file already exists
    echo [?] Do you want to overwrite it? (Y/N)
    choice /C YN /N
    if errorlevel 2 goto skip_env
)

echo [*] Creating .env file from template...
copy .env.example .env
echo [OK] .env file created
echo.
echo [!] IMPORTANT: Please edit backend\.env and add your API keys:
echo     - OPENAI_API_KEY (from https://platform.openai.com/)
echo     - QDRANT_URL and QDRANT_API_KEY (from https://cloud.qdrant.io/)
echo     - DATABASE_URL (from https://neon.tech/)
echo     - JWT_SECRET_KEY (run: openssl rand -hex 32)
echo.
echo Press any key to continue after you've added your keys...
pause >nul

:skip_env
cd ..

REM Step 3: Install frontend dependencies
echo ========================================
echo  Step 3/4: Setting up Frontend
echo ========================================
echo.

if exist "node_modules\" (
    echo [!] Node modules already exist, skipping...
) else (
    echo [*] Installing frontend dependencies (this may take 5-10 minutes)...
    call npm install
    echo [OK] Frontend dependencies installed
)
echo.

REM Step 4: Initialize databases
echo ========================================
echo  Step 4/4: Database Setup
echo ========================================
echo.

echo [?] Do you want to run database migrations now? (Y/N)
choice /C YN /N
if errorlevel 2 goto skip_db

cd backend
call venv\Scripts\activate

echo [*] Running PostgreSQL migrations...
python database\run_migrations.py
echo [OK] Migrations complete

echo [*] Setting up Qdrant collection...
python database\setup_qdrant.py
echo [OK] Qdrant setup complete

echo [*] Ingesting textbook content...
python ingest_content.py
echo [OK] Content ingestion complete

cd ..

:skip_db
echo.
echo ========================================
echo  Setup Complete!
echo ========================================
echo.
echo [OK] All dependencies installed
echo [OK] Environment configured
echo.
echo Next steps:
echo   1. Make sure backend\.env has all your API keys
echo   2. Double-click START.bat to run the application
echo   3. Visit http://localhost:3000 to see the textbook
echo.
echo Press any key to exit...
pause >nul
