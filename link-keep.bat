@echo off
setlocal
title Meowler link-keep
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\link-keep.ps1"
endlocal
