# BugTracker 一键停止：优先按 pid 文件，兜底按端口找进程（可停前台/后台任意模式）
# 用法：.\stop.ps1
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$pidFile = Join-Path $root "data\server.pid"

# 从 .env 读端口（与 ensure-ready.ps1 同规则），兜底默认 8500
$port = 8500
$envFile = Join-Path $root ".env"
if (Test-Path $envFile) {
    foreach ($line in (Get-Content $envFile)) {
        if ($line -match '^\s*BT_PORT\s*=\s*(\d+)') { $port = [int]$Matches[1] }
    }
}

# 只杀命令行含 -m bugtracker 的 python，防止误杀同 pid/同端口的无关进程
function Stop-BtProcess([int]$ProcId) {
    $proc = Get-CimInstance Win32_Process -Filter "ProcessId=$ProcId" -ErrorAction SilentlyContinue
    if (-not $proc) { return $false }
    if ($proc.CommandLine -notmatch "bugtracker") {
        Write-Host "[BugTracker] PID $ProcId 不是 BugTracker 进程，跳过（防误杀）"
        return $false
    }
    Stop-Process -Id $ProcId -Force
    return $true
}

$stopped = $false
if (Test-Path $pidFile) {
    $oldPid = [int](Get-Content $pidFile).Trim()
    $stopped = Stop-BtProcess $oldPid
    Remove-Item $pidFile -Force -ErrorAction SilentlyContinue
}
if (-not $stopped) {
    # 兜底：谁占着端口谁就是服务（覆盖前台模式、pid 文件丢失等场景）
    $conn = Get-NetTCPConnection -LocalPort $port -State Listen -ErrorAction SilentlyContinue |
            Select-Object -First 1
    if ($conn) { $stopped = Stop-BtProcess $conn.OwningProcess }
}

if (-not $stopped) {
    Write-Host "[BugTracker] 未发现运行中的服务（端口 $port 空闲）"
    exit 0
}

# 等端口真正释放
foreach ($i in 1..20) {
    Start-Sleep -Milliseconds 500
    if (-not (Get-NetTCPConnection -LocalPort $port -State Listen -ErrorAction SilentlyContinue)) { break }
}
Write-Host "[BugTracker] 已停止（端口 $port 已释放，数据保留在 data\）"
