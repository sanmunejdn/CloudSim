# 把 BugTracker 的 post-commit hook 安装到指定 git 仓库。
# 用法：.\install_hook.ps1 -RepoPath d:\Project\VSprogram\CGAL5.5.2\CloudSim [-Api http://127.0.0.1:8500]
param(
    [Parameter(Mandatory = $true)][string]$RepoPath,
    [string]$Api = "http://127.0.0.1:8500"
)

$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$gitDir = Join-Path $RepoPath ".git"
if (-not (Test-Path $gitDir)) { throw "不是 git 仓库: $RepoPath" }

# 从 .env 读 token
$envFile = Join-Path (Split-Path -Parent $here) ".env"
$token = ""
if (Test-Path $envFile) {
    foreach ($line in Get-Content $envFile) {
        if ($line -match '^\s*BT_INTEGRATION_TOKEN\s*=\s*(.+)$') { $token = $Matches[1].Trim() }
    }
}
if (-not $token) { throw "未在 .env 找到 BT_INTEGRATION_TOKEN，请先配置" }

$hooksDir = Join-Path $gitDir "hooks"
New-Item -ItemType Directory -Force -Path $hooksDir | Out-Null
$hookPath = Join-Path $hooksDir "post-commit"
$scanPath = (Join-Path $here "scan_commits.py") -replace '\\', '/'

$content = @"
#!/bin/sh
# BugTracker post-commit hook（由 install_hook.ps1 生成）
python "$scanPath" --api "$Api" --token "$token" >/dev/null 2>&1 &
exit 0
"@
Set-Content -Path $hookPath -Value $content -Encoding ASCII
Write-Host "[BugTracker] hook 已安装: $hookPath"
Write-Host "[BugTracker] commit message 中 'fix #ID' 将自动把对应 bug 置为已修复。"
