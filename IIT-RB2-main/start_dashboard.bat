@echo off
title Robot 2 Dashboard - Root Launcher
cd /d "%~dp0"

echo ========================================================
echo   IIT-RB2: ROBOT 2 GROUND CONTROL STATION LAUNCHER
echo ========================================================
echo Starting HTTP & Serial Bridge Server on port 8080...
start "" "http://localhost:8080/"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0dashboard\serve_dashboard.ps1" -Port 8080 -RootPath "%~dp0dashboard"
pause
