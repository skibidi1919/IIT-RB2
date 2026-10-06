@echo off
setlocal
REM Deploy app/*.py via mpremote
REM Usage: deploy_app.bat [COMx]

set PORT=%~1
if "%PORT%"=="" set PORT=auto

set ROOT=%~dp0..
set APP=%ROOT%\app

where mpremote >nul 2>&1
if errorlevel 1 (
  python -m pip install -r "%ROOT%\requirements-host.txt"
)

for %%F in (boot.py main.py pinout.py pca9685.py vl53.py drive.py color_tcs.py protocol.py ota_http.py) do (
  echo %%F
  mpremote connect %PORT% cp "%APP%\%%F" ":%%F"
)

if exist "%APP%\secrets.py" (
  echo secrets.py
  mpremote connect %PORT% cp "%APP%\secrets.py" ":secrets.py"
) else (
  mpremote connect %PORT% cp "%APP%\secrets.py.example" ":secrets.py.example"
  echo Create app\secrets.py then redeploy.
)

mpremote connect %PORT% exec "import machine; machine.reset()"
echo Done.
endlocal
