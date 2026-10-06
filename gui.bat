@echo off
setlocal EnableExtensions

rem Meowler GUI launcher
rem   gui.bat           start (default COM3)
rem   gui.bat COM4      start on COM4
rem   gui.bat stop      stop server + free COM port

set "GUI_DIR=%~dp0gui"
set "UV=%USERPROFILE%\.local\bin\uv.exe"
set "URL=http://127.0.0.1:5050"

if /i "%~1"=="stop" goto stop
if /i "%~1"=="kill" goto stop

set "PORT=%~1"
if "%PORT%"=="" set "PORT=COM3"

if not exist "%UV%" (
  echo ERROR: uv not found at "%UV%"
  exit /b 1
)

cd /d "%GUI_DIR%"

rem If already up, just open browser / reconnect
curl -fsS --max-time 1 "%URL%/api/status" >nul 2>&1
if not errorlevel 1 (
  echo Meowler GUI already running -^> %URL%
  start "" "%URL%"
  "%UV%" run python -c "import urllib.request,json; urllib.request.urlopen(urllib.request.Request('%URL%/api/connect', data=json.dumps({'port':'%PORT%','baud':115200}).encode(), headers={'Content-Type':'application/json'}, method='POST'), timeout=20).read()" 2>nul
  exit /b 0
)

echo Starting Meowler GUI ...
start "Meowler GUI" /MIN "%UV%" run python app.py

set /a _n=0
:wait
set /a _n+=1
curl -fsS --max-time 1 "%URL%/api/status" >nul 2>&1
if not errorlevel 1 goto ready
if %_n% GEQ 40 (
  echo Failed to start GUI. Is uv/python OK?
  exit /b 1
)
ping -n 1 127.0.0.1 >nul
goto wait

:ready
echo Connecting %PORT% ...
"%UV%" run python -c "import urllib.request,json; urllib.request.urlopen(urllib.request.Request('%URL%/api/connect', data=json.dumps({'port':'%PORT%','baud':115200}).encode(), headers={'Content-Type':'application/json'}, method='POST'), timeout=20).read(); print('connected')"
start "" "%URL%"
echo Meowler GUI -^> %URL%
endlocal
exit /b 0

:stop
echo Stopping Meowler GUI ...
rem Prefer clean disconnect so COM is released
curl -fsS --max-time 2 -X POST "%URL%/api/disconnect" -H "Content-Type: application/json" -d "{}" >nul 2>&1

rem Kill listeners on port 5050
for /f "tokens=5" %%P in ('netstat -ano ^| findstr /R /C:":5050 .*LISTENING"') do (
  echo Killing PID %%P
  taskkill /F /PID %%P >nul 2>&1
)

rem Also kill titled window / leftover uv/python for this app
taskkill /F /FI "WINDOWTITLE eq Meowler GUI*" >nul 2>&1

echo Meowler GUI stopped.
endlocal
exit /b 0
