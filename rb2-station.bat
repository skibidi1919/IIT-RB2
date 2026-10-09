@echo off
setlocal
title RB2 — Youth Challenge Mission Station

cd /d "%~dp0\rb2_ui"

echo =======================================================
echo    RB2 — Robot 2 Autonomous Mission Control Station
echo =======================================================
echo.

set "UV_PATH=%USERPROFILE%\.local\bin\uv.exe"

if exist "%UV_PATH%" (
    echo Launching via uv: %UV_PATH%
    start "" http://127.0.0.1:5055
    "%UV_PATH%" run --with flask --with pyserial python app.py --http-port 5055
    goto done
)

where uv >nul 2>nul
if %errorlevel% equ 0 (
    echo Launching via uv...
    start "" http://127.0.0.1:5055
    uv run --with flask --with pyserial python app.py --http-port 5055
    goto done
)

where py >nul 2>nul
if %errorlevel% equ 0 (
    echo Launching via py...
    start "" http://127.0.0.1:5055
    py -m pip install flask pyserial >nul 2>nul
    py app.py --http-port 5055
    goto done
)

where python >nul 2>nul
if %errorlevel% equ 0 (
    echo Launching via python...
    start "" http://127.0.0.1:5055
    python app.py --http-port 5055
    goto done
)

echo.
echo [ERROR] Neither uv nor python could be found.
echo Please ensure Python or uv is in your PATH.

:done
pause
