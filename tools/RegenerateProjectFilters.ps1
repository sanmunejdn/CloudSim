#Requires -Version 5.1
<#
.SYNOPSIS
  Regenerate CloudSim *.vcxproj.filters by functional rules.

.DESCRIPTION
  权威入口已迁至 Python（与 SOURCE_CONVENTIONS / Cursor 规则一致）：
    python scripts/generate_vcxproj_filters.py --full
    python scripts/generate_vcxproj_filters.py --sync --project <Name>
  本脚本作为兼容包装，转发到上述命令。
#>
param(
  [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path,
  [string[]]$OnlyProjects = @(),
  [switch]$Sync
)

$ErrorActionPreference = 'Stop'
$py = Join-Path $Root 'scripts\generate_vcxproj_filters.py'
if (-not (Test-Path -LiteralPath $py)) { throw "Missing: $py" }

$fwd = @($py)
if ($Sync) { $fwd += '--sync' } else { $fwd += '--full' }
foreach ($p in $OnlyProjects) {
  foreach ($n in ($p -split ',')) {
    $t = $n.Trim()
    if ($t) { $fwd += @('--project', $t) }
  }
}

Write-Host "Forwarding: python $($fwd -join ' ')"
& python @fwd
exit $LASTEXITCODE
