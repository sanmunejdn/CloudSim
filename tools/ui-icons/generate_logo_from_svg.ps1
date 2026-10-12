# 用 Qt QSvgRenderer 将 cloudsim_logo.svg 栅格化为 light/dark 各尺寸 PNG
# 依赖：VS 2019+ 开发者环境、D:\Qt\Qt5.14.2\5.14.2\msvc2017_64（与工程 QtInstall 一致）

$ErrorActionPreference = 'Stop'
$repoUiIcons = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoCloudSim = (Resolve-Path (Join-Path $repoUiIcons '..\..')).Path
$qt = 'D:\Qt\Qt5.14.2\5.14.2\msvc2017_64'
$svg = Join-Path $repoCloudSim 'src\UI\CloudSimUiAssets\resources\logo\cloudsim_logo.svg'
$outDir = Join-Path $repoCloudSim 'src\UI\CloudSimUiAssets\resources\logo'
$exe = Join-Path $repoUiIcons 'rasterize_logo_qt.exe'
$src = Join-Path $repoUiIcons 'rasterize_logo_qt.cpp'

if (-not (Test-Path $svg)) { throw "missing svg: $svg" }
if (-not (Test-Path "$qt\bin\Qt5Svg.dll")) { throw "Qt Svg not found: $qt" }

$vsPath = & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property installationPath
Import-Module "$vsPath\Common7\Tools\Microsoft.VisualStudio.DevShell.dll"
Enter-VsDevShell -VsInstallPath $vsPath -SkipAutomaticLocation -DevCmdArguments '-arch=x64' | Out-Null

$inc = @("/I$qt\include", "/I$qt\include\QtCore", "/I$qt\include\QtGui", "/I$qt\include\QtSvg", "/I$qt\include\QtWidgets")
$libs = @("$qt\lib\Qt5Core.lib", "$qt\lib\Qt5Gui.lib", "$qt\lib\Qt5Svg.lib", "$qt\lib\Qt5Widgets.lib")
& cl /nologo /EHsc /std:c++17 /MD /O2 /DNDEBUG /DQT_NO_DEBUG @inc $src /Fe:$exe /link /LIBPATH:"$qt\lib" @libs /SUBSYSTEM:CONSOLE
if ($LASTEXITCODE -ne 0) { throw "cl failed: $LASTEXITCODE" }

$env:PATH = "$qt\bin;" + $env:PATH
& $exe $svg $outDir
if ($LASTEXITCODE -ne 0) { throw "rasterize failed: $LASTEXITCODE" }
Write-Host "OK: logo PNGs under $outDir"
