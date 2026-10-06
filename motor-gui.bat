@echo off
setlocal
set "PATH=%USERPROFILE%\.local\bin;%PATH%"
set PYTHONIOENCODING=utf-8
cd /d "%~dp0motor_gui"
uv sync
uv run python app.py
endlocal
