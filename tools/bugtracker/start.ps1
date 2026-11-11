# BugTracker 一键后台启动：进程独立于启动终端，关闭 Cursor/终端不影响
# 用法：.\start.ps1    停止用 .\stop.ps1
$ErrorActionPreference = "Stop"
. (Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Path) "ensure-ready.ps1")

$pidFile = Join-Path $BtDataDir "server.pid"

# Win11 上部分环境 State 为空，-State Listen 恒匹配不到；用 RemotePort=0 识别监听套接字
function Get-BtListeners([int]$Port) {
    Get-NetTCPConnection -LocalPort $Port -ErrorAction SilentlyContinue |
        Where-Object { $_.RemotePort -eq 0 }
}

function Test-BtCommandLine([int]$ProcId) {
    $proc = Get-CimInstance Win32_Process -Filter "ProcessId=$ProcId" -ErrorAction SilentlyContinue
    return ($proc -and $proc.CommandLine -match "bugtracker")
}

function Test-BtProcessTreeOwnsPort([int]$RootPid, [int]$Port) {
    $owner = (Get-BtListeners $Port | Select-Object -First 1).OwningProcess
    if (-not $owner) { return $false }
    if ($owner -eq $RootPid) { return $true }
    $node = Get-CimInstance Win32_Process -Filter "ProcessId=$owner" -ErrorAction SilentlyContinue
    while ($node) {
        if ($node.ProcessId -eq $RootPid) { return $true }
        if (-not $node.ParentProcessId -or $node.ParentProcessId -eq 0) { break }
        $node = Get-CimInstance Win32_Process -Filter "ProcessId=$($node.ParentProcessId)" -ErrorAction SilentlyContinue
    }
    return $false
}

# 已在运行则直接报告（pid 文件 + 端口属主，防止 pid 复用误判）
if (Test-Path $pidFile) {
    $oldPid = [int](Get-Content $pidFile).Trim()
    if ((Test-BtCommandLine $oldPid) -and (Test-BtProcessTreeOwnsPort $oldPid $BtPort)) {
        Write-Host "[BugTracker] 已在运行（PID $oldPid）：http://127.0.0.1:$BtPort"
        exit 0
    }
    Remove-Item $pidFile -Force -ErrorAction SilentlyContinue
}

$existing = Get-BtListeners $BtPort | Select-Object -First 1
if ($existing) {
    $owner = [int]$existing.OwningProcess
    if (Test-BtCommandLine $owner) {
        Set-Content $pidFile $owner -Encoding ASCII
        Write-Host "[BugTracker] 已在运行（PID $owner）：http://127.0.0.1:$BtPort"
        exit 0
    }
    Write-Host "[BugTracker] 端口 $BtPort 被占用（PID $owner）且非 BugTracker，先释放端口或改 .env 中 BT_PORT"
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

# 轮询健康检查，且确认端口属主属于本次进程树（避免旧实例健康响应冒充成功）
$ok = $false
foreach ($i in 1..40) {
    Start-Sleep -Milliseconds 500
    if (-not (Get-Process -Id $proc.Id -ErrorAction SilentlyContinue)) {
        break
    }
    try {
        $r = Invoke-RestMethod "http://127.0.0.1:$BtPort/api/health" -TimeoutSec 2
        if ($r.ok -and (Test-BtProcessTreeOwnsPort $proc.Id $BtPort)) {
            $ok = $true
            break
        }
    } catch { }
}
if (-not $ok) {
    if (Get-Process -Id $proc.Id -ErrorAction SilentlyContinue) {
        Stop-Process -Id $proc.Id -Force -ErrorAction SilentlyContinue
    }
    Remove-Item $pidFile -Force -ErrorAction SilentlyContinue
    Write-Host "[BugTracker] 启动失败，日志见：$errLog"
    if (Test-Path $errLog) {
        Get-Content $errLog -Tail 12 | ForEach-Object { Write-Host $_ }
    }
    exit 1
}
Write-Host "[BugTracker] 已后台启动（PID $($proc.Id)）：本机 http://127.0.0.1:$BtPort ，局域网 http://<本机IP>:$BtPort"
Write-Host "[BugTracker] 停止：.\stop.ps1 ；运行日志：data\server.out.log"
