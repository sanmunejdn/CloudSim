# BugTracker 前台启动（调试用，日志直接打在控制台，Ctrl+C 停止）
# 日常使用推荐：.\start.ps1 后台启动 / .\stop.ps1 停止（脱离终端存活）
$ErrorActionPreference = "Stop"
. (Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Path) "ensure-ready.ps1")

if (Get-NetTCPConnection -LocalPort $BtPort -State Listen -ErrorAction SilentlyContinue) {
    Write-Host "[BugTracker] 端口 $BtPort 已被占用（服务可能已在运行）：http://127.0.0.1:$BtPort"
    exit 1
}

Write-Host "[BugTracker] 前台启动：http://127.0.0.1:$BtPort （Ctrl+C 停止）"
Write-Host "[BugTracker] 首次启动会在控制台打印 admin 初始密码（仅一次），请记录。"
& $BtPython -m bugtracker
