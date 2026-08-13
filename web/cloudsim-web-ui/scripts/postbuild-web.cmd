@echo off
REM CloudSimWeb PostBuild: Vite output under bin\x64d\web or bin\x64\web
REM CLOUDSIM_WEB_SKIP_BUILD=1 skips Vite
REM CLOUDSIM_WEB_FALLBACK=1 uses _archive\public-fallback
setlocal EnableExtensions EnableDelayedExpansion
set "UI_ROOT=%~dp0.."
set "OUT_WEB=%~1"
set "BUILD_MODE=%~2"
if "!OUT_WEB!"=="" (
  echo [cloudsim-web] missing OutDir web path
  exit /b 1
)
REM Reject unexpanded MSBuild macros so we never mkdir under the vcxproj dir
set "HAS_MACRO=0"
echo.!OUT_WEB!| findstr /C:"$(" >nul 2>&1 && set "HAS_MACRO=1"
if "!HAS_MACRO!"=="1" (
  echo [cloudsim-web] refusing unexpanded macro path: !OUT_WEB!
  echo [cloudsim-web] expected expanded path from CloudSimWeb PostBuild CloudSimBinDir\web
  exit /b 1
)
if /I "!CLOUDSIM_WEB_SKIP_BUILD!"=="1" (
  echo [cloudsim-web] SKIP_BUILD=1, leave !OUT_WEB! unchanged
  exit /b 0
)
if not exist "!OUT_WEB!" mkdir "!OUT_WEB!"
if errorlevel 1 (
  echo [cloudsim-web] mkdir failed: !OUT_WEB!
  exit /b 1
)
if /I "!CLOUDSIM_WEB_FALLBACK!"=="1" (
  echo [cloudsim-web] FALLBACK=1, xcopy _archive\public-fallback
  if not exist "!UI_ROOT!\_archive\public-fallback\" (
    echo [cloudsim-web] archive missing: !UI_ROOT!\_archive\public-fallback
    exit /b 1
  )
  xcopy /Y /I /Q /E "!UI_ROOT!\_archive\public-fallback\*" "!OUT_WEB!\"
  exit /b !ERRORLEVEL!
)

set "NPM=npm"
where npm >nul 2>&1
if errorlevel 1 (
  if exist "!UI_ROOT!\.tools\node\npm.cmd" (
    set "PATH=!UI_ROOT!\.tools\node;!PATH!"
    set "NPM=npm.cmd"
  ) else (
    echo [cloudsim-web] npm not found. Install Node 18+ or place portable node in web\cloudsim-web-ui\.tools\node
    echo [cloudsim-web] or set CLOUDSIM_WEB_FALLBACK=1 / CLOUDSIM_WEB_SKIP_BUILD=1
    exit /b 1
  )
)
pushd "!UI_ROOT!"
if not exist "node_modules\" (
  echo [cloudsim-web] npm install...
  call !NPM! ci 2>nul
  if errorlevel 1 call !NPM! install
  if errorlevel 1 (
    popd
    exit /b 1
  )
)
if /I "!BUILD_MODE!"=="release" (
  call !NPM! run build:release
) else (
  call !NPM! run build:debug
)
set "EC=!ERRORLEVEL!"
popd
if not "!EC!"=="0" (
  echo [cloudsim-web] vite build failed EC=!EC!
  exit /b !EC!
)
echo [cloudsim-web] built to !OUT_WEB!
exit /b 0
