@echo off
REM ==============================================================================
REM Physical AI Textbook - Stop All Services (Windows)
REM ==============================================================================
REM This script stops all running Node.js and Python processes
REM Use this if you closed windows without stopping servers properly
REM ==============================================================================

echo.
echo ========================================
echo  Stopping All Services...
echo ========================================
echo.

REM Kill all Node.js processes (Docusaurus)
echo [*] Stopping Node.js processes...
taskkill /F /IM node.exe >nul 2>&1
if %errorlevel% equ 0 (
    echo [OK] Node.js processes stopped
) else (
    echo [!] No Node.js processes found
)

REM Kill all Python processes (FastAPI)
echo [*] Stopping Python processes...
taskkill /F /IM python.exe >nul 2>&1
if %errorlevel% equ 0 (
    echo [OK] Python processes stopped
) else (
    echo [!] No Python processes found
)

echo.
echo ========================================
echo  All Services Stopped
echo ========================================
echo.
echo Press any key to exit...
pause >nul
