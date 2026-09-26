# 跑 API pytest；默认 Debug → bin\x64d

param(
    [Parameter(Mandatory = $false)]
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Debug',

    [Parameter(Mandatory = $false)]
    [string]$PytestArgs = ''
)

$ErrorActionPreference = 'Stop'
$CloudSimRoot = Split-Path -Parent $PSScriptRoot
Set-Location $CloudSimRoot

$python = Get-Command python -ErrorAction SilentlyContinue
if (-not $python) {
    Write-Error 'python not found in PATH'
    exit 1
}

$req = Join-Path $CloudSimRoot 'tests\requirements.txt'
& python -m pip install -q -r $req
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$env:CLOUDSIM_TEST_CONFIG = $Configuration
$junit = Join-Path $CloudSimRoot "artifacts\checks\api-$Configuration-junit.xml"
New-Item -ItemType Directory -Force -Path (Split-Path $junit) | Out-Null

$argsList = @(
    '-m', 'pytest',
    'tests\api',
    '-v',
    "--cloudsim-config=$Configuration",
    "--junitxml=$junit"
)
if ($PytestArgs) {
    $argsList += $PytestArgs.Split(' ')
}

& python @argsList
exit $LASTEXITCODE
