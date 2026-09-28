# 安装 pre-commit：backend() 穿透棘轮
param([Parameter(Mandatory = $true)][string]$RepoPath)
$ErrorActionPreference = "Stop"
$gitDir = Join-Path $RepoPath ".git"
if (-not (Test-Path $gitDir)) { throw "不是 git 仓库: $RepoPath" }
$hooksDir = Join-Path $gitDir "hooks"
New-Item -ItemType Directory -Force -Path $hooksDir | Out-Null
$hookPath = Join-Path $hooksDir "pre-commit"
$cloudSim = Join-Path $RepoPath "CloudSim"
if (-not (Test-Path (Join-Path $cloudSim "scripts\check_backend_callsites.py"))) {
    $cloudSim = $RepoPath
}
$script = (Join-Path $cloudSim "scripts\check_backend_callsites.py") -replace '\\', '/'
$block = @"

# CloudSim backend() ratchet
python "$script" || exit 1
"@
if (Test-Path $hookPath) {
    $existing = Get-Content $hookPath -Raw
    if ($existing -notmatch "check_backend_callsites") {
        Add-Content -Path $hookPath -Value $block -Encoding ASCII
    }
} else {
    Set-Content -Path $hookPath -Value "#!/bin/sh`n$block" -Encoding ASCII
}
Write-Host "Installed backend callsites check into $hookPath"
