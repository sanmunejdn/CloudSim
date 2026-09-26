@echo off
REM Double-click: Debug + Release SelfTest(gate) + API tests
cd /d "%~dp0"
title CloudSim one-click tests
chcp 65001 >nul
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\run_tests.ps1" -Configuration Both %*
set ERR=%ERRORLEVEL%
echo.
if %ERR% neq 0 (
  echo [FAILED] exit %ERR%
) else (
  echo [OK] All passed Debug+Release
)
pause
exit /b %ERR%
