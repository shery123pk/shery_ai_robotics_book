@echo off
echo ========================================
echo  Restarting Frontend with Fresh Build
echo ========================================
echo.

REM Kill any process on port 3000
for /f "tokens=5" %%a in ('netstat -ano ^| findstr :3000') do taskkill /F /PID %%a 2>nul

echo [*] Clearing Docusaurus cache...
call npm run clear

echo [*] Starting frontend on http://localhost:3000...
echo.
echo [!] Backend must be running at http://localhost:8000
echo.

start "Physical AI Frontend" cmd /k "npm start"

echo.
echo [SUCCESS] Frontend is starting in a new window!
echo.
echo Visit: http://localhost:3000
echo Test chatbot: Click the chat icon in the bottom right
echo.
pause
