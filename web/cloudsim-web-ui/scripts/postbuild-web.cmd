@echo off
REM CloudSimWeb PostBuild: Vite -> $(OutDir)web
REM CLOUDSIM_WEB_SKIP_BUILD=1  跳过（纯 C++ 迭代）
REM CLOUDSIM_WEB_FALLBACK=1    临时回退 xcopy _archive\public-fallback
setlocal EnableExtensions
set "UI_ROOT=%~dp0.."
set "OUT_WEB=%~1"
set "BUILD_MODE=%~2"
if "%OUT_WEB%"=="" (
  echo [cloudsim-web] missing OutDir web path
  exit /b 1
)
if /I "%CLOUDSIM_WEB_SKIP_BUILD%"=="1" (
  echo [cloudsim-web] SKIP_BUILD=1, leave %OUT_WEB% unchanged
  exit /b 0
)
if not exist "%OUT_WEB%" mkdir "%OUT_WEB%"
if /I "%CLOUDSIM_WEB_FALLBACK%"=="1" (
  echo [cloudsim-web] FALLBACK=1, xcopy _archive\public-fallback
  if not exist "%UI_ROOT%\_archive\public-fallback\" (
    echo [cloudsim-web] archive missing: %UI_ROOT%\_archive\public-fallback
    exit /b 1
  )
  xcopy /Y /I /Q /E "%UI_ROOT%\_archive\public-fallback\*" "%OUT_WEB%\"
  exit /b %ERRORLEVEL%
)

REM Prefer PATH npm; else portable .tools\node
set "NPM=npm"
where npm >nul 2>&1
if errorlevel 1 (
  if exist "%UI_ROOT%\.tools\node\npm.cmd" (
    set "PATH=%UI_ROOT%\.tools\node;%PATH%"
    set "NPM=npm.cmd"
  ) else (
    echo [cloudsim-web] npm not found. Install Node 18+ or place portable node in web\cloudsim-web-ui\.tools\node
    echo [cloudsim-web] or set CLOUDSIM_WEB_FALLBACK=1 / CLOUDSIM_WEB_SKIP_BUILD=1
    exit /b 1
  )
)
pushd "%UI_ROOT%"
if not exist "node_modules\" (
  echo [cloudsim-web] npm install...
  call %NPM% ci 2>nul
  if errorlevel 1 call %NPM% install
  if errorlevel 1 (
    popd
    exit /b 1
  )
)
if /I "%BUILD_MODE%"=="release" (
  call %NPM% run build:release
) else (
  call %NPM% run build:debug
)
set "EC=%ERRORLEVEL%"
popd
if not "%EC%"=="0" (
  echo [cloudsim-web] vite build failed ^(%EC%^)
  exit /b %EC%
)
echo [cloudsim-web] built -^> %OUT_WEB%
exit /b 0
