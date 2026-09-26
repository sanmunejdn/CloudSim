# soak 稳定性循环

param(
    [Parameter(Mandatory = $false)]
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Debug',

    [Parameter(Mandatory = $false)]
    [int]$Rounds = 200,

    [Parameter(Mandatory = $false)]
    [double]$MaxMbPerRound = 0.5
)

$ErrorActionPreference = 'Stop'
$CloudSimRoot = Split-Path -Parent $PSScriptRoot
Set-Location $CloudSimRoot

& python -m pip install -q -r (Join-Path $CloudSimRoot 'tests\requirements.txt')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& python (Join-Path $CloudSimRoot 'tests\soak\soak_loop.py') `
    --configuration $Configuration `
    --rounds $Rounds `
    --max-mb-per-round $MaxMbPerRound
exit $LASTEXITCODE
