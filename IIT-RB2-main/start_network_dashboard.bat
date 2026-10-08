@echo off
title Robot 2 Dashboard - Network Server
cd /d "%~dp0"

echo Starting Robot 2 Dashboard Network Server...
start "" "http://localhost:8080/"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0serve_dashboard.ps1" -Port 8080
pause
