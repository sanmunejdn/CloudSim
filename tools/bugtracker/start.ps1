# BugTracker 一键后台启动：进程独立于启动终端，关闭 Cursor/终端不影响
# 用法：.\start.ps1    停止用 .\stop.ps1
$ErrorActionPreference = "Stop"
. (Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Path) "ensure-ready.ps1")

$pidFile = Join-Path $BtDataDir "server.pid"

# 已在运行则直接报告（pid 文件 + 命令行双重确认，防止 pid 复用误判）
if (Test-Path $pidFile) {
    $oldPid = [int](Get-Content $pidFile).Trim()
    $proc = Get-CimInstance Win32_Process -Filter "ProcessId=$oldPid" -ErrorAction SilentlyContinue
    if ($proc -and $proc.CommandLine -match "bugtracker") {
        Write-Host "[BugTracker] 已在运行（PID $oldPid）：http://127.0.0.1:$BtPort"
        exit 0
    }
    Remove-Item $pidFile -Force
}
if (Get-NetTCPConnection -LocalPort $BtPort -State Listen -ErrorAction SilentlyContinue) {
    Write-Host "[BugTracker] 端口 $BtPort 被占用且无 pid 记录（可能是前台模式在跑），先执行 .\stop.ps1"
    exit 1
}

New-Item -ItemType Directory -Force -Path $BtDataDir | Out-Null
$outLog = Join-Path $BtDataDir "server.out.log"
$errLog = Join-Path $BtDataDir "server.err.log"

# Start-Process 重定向的 out/err 无内置轮转；超过 10MB 保留一代
foreach ($log in @($outLog, $errLog)) {
    if ((Test-Path $log) -and ((Get-Item $log).Length -gt 10MB)) {
        Move-Item $log ($log -replace '\.log$', '.1.log') -Force
    }
}

# 隐藏窗口 = 独立新控制台，不附着 Cursor 伪终端，父进程退出后存活
$proc = Start-Process -FilePath $BtPython -ArgumentList "-m", "bugtracker" `
    -WorkingDirectory $BtRoot -WindowStyle Hidden -PassThru `
    -RedirectStandardOutput $outLog -RedirectStandardError $errLog
Set-Content $pidFile $proc.Id -Encoding ASCII

# 轮询健康检查，确认服务真正可用
$ok = $false
foreach ($i in 1..40) {
    Start-Sleep -Milliseconds 500
    try {
        $r = Invoke-RestMethod "http://127.0.0.1:$BtPort/api/health" -TimeoutSec 2
        if ($r.ok) { $ok = $true; break }
    } catch { }
}
if (-not $ok) {
    Write-Host "[BugTracker] 启动超时，日志见：$errLog"
    exit 1
}
Write-Host "[BugTracker] 已后台启动（PID $($proc.Id)）：本机 http://127.0.0.1:$BtPort ，局域网 http://<本机IP>:$BtPort"
Write-Host "[BugTracker] 停止：.\stop.ps1 ；运行日志：data\server.out.log"
