# BugTracker 环境就绪检查（供 run.ps1 / start.ps1 dot-source，勿直接运行）
# 完成后提供：$BtRoot $BtPython $BtPort $BtDataDir
$ErrorActionPreference = "Stop"
$BtRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $BtRoot

$venv = Join-Path $BtRoot ".venv"
$BtPython = Join-Path $venv "Scripts\python.exe"
$BtDataDir = Join-Path $BtRoot "data"

if (-not (Test-Path $BtPython)) {
    Write-Host "[BugTracker] 创建虚拟环境 .venv ..."
    python -m venv $venv
    if ($LASTEXITCODE -ne 0) { throw "venv 创建失败，请确认系统 python 可用" }
}

& $BtPython -c "import fastapi, uvicorn, sqlalchemy, mcp" 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Host "[BugTracker] 安装依赖（首次较慢）..."
    & $BtPython -m pip install -r (Join-Path $BtRoot "requirements.txt")
    if ($LASTEXITCODE -ne 0) { throw "依赖安装失败；离线环境请参考 README 的离线安装章节" }
}

$envFile = Join-Path $BtRoot ".env"
if (-not (Test-Path $envFile)) {
    Copy-Item (Join-Path $BtRoot ".env.example") $envFile
    $token = -join ((48..57) + (97..122) | Get-Random -Count 32 | ForEach-Object { [char]$_ })
    (Get-Content $envFile) -replace "BT_INTEGRATION_TOKEN=.*", "BT_INTEGRATION_TOKEN=$token" |
        Set-Content $envFile
    Write-Host "[BugTracker] 已生成 .env（含随机集成令牌；SMTP 等按需修改）"
}

$BtPort = 8500
foreach ($line in (Get-Content $envFile)) {
    if ($line -match '^\s*BT_PORT\s*=\s*(\d+)') { $BtPort = [int]$Matches[1] }
}
