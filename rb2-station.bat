@echo off
title RB2 — Youth Challenge Mission Station
cd /d "%~dp0\rb2_ui"

echo =======================================================
echo    RB2 — Robot 2 Autonomous Mission Control Station
echo =======================================================
echo.
echo Starting Web UI server on http://127.0.0.1:5055 ...
echo.

start "" http://127.0.0.1:5055

python app.py --port 5055
if errorlevel 1 (
    echo.
    echo Trying with uv run...
    uv run --with flask --with pyserial python app.py --port 5055
)
pause
