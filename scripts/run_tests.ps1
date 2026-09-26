# One-click tests (default: Debug + Release)
# Usage: .\scripts\run_tests.ps1
#        .\scripts\run_tests.ps1 -Configuration Debug
#        .\scripts\run_tests.ps1 -FullBuild
#        .\scripts\run_tests.ps1 -WithSoak -SoakRounds 20

param(
    [Parameter(Mandatory = $false)]
    [ValidateSet('Debug', 'Release', 'Both')]
    [string]$Configuration = 'Both',

    [Parameter(Mandatory = $false)]
    [switch]$FullBuild,

    [Parameter(Mandatory = $false)]
    [switch]$WithSoak,

    [Parameter(Mandatory = $false)]
    [int]$SoakRounds = 20
)

$ErrorActionPreference = 'Stop'
$CloudSimRoot = Split-Path -Parent $PSScriptRoot
Set-Location $CloudSimRoot

function Find-MsBuild {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $p = & $vswhere -latest -requires Microsoft.Component.MSBuild -find 'MSBuild\**\Bin\MSBuild.exe' | Select-Object -First 1
        if ($p) { return $p }
    }
    return 'msbuild'
}

function Get-BinDir([string]$cfg) {
    $rel = if ($cfg -eq 'Debug') { '..\bin\x64d' } else { '..\bin\x64' }
    return [System.IO.Path]::GetFullPath((Join-Path $CloudSimRoot $rel))
}

function Ensure-Binaries([string]$cfg) {
    $bin = Get-BinDir $cfg
    $runner = Join-Path $bin 'SelfTestRunner.exe'
    $web = Join-Path $bin 'CloudSimWeb.exe'
    $needBuild = $FullBuild -or (-not (Test-Path $runner)) -or (-not (Test-Path $web))
    if (-not $needBuild) {
        Write-Host ("  binaries OK: {0}" -f $bin)
        return
    }
    Write-Host ("  build SelfTestRunner + CloudSimWeb ({0}) ..." -f $cfg) -ForegroundColor Yellow
    $msb = Find-MsBuild
    & $msb (Join-Path $CloudSimRoot 'src\App\SelfTestRunner\SelfTestRunner.vcxproj') `
        /p:Configuration=$cfg /p:Platform=x64 /m
    if ($LASTEXITCODE -ne 0) {
        Write-Host ("SelfTestRunner build FAILED ({0})" -f $cfg) -ForegroundColor Red
        exit $LASTEXITCODE
    }
    & $msb (Join-Path $CloudSimRoot 'src\App\CloudSimWeb\CloudSimWeb.vcxproj') `
        /p:Configuration=$cfg /p:Platform=x64 /m
    if ($LASTEXITCODE -ne 0) {
        Write-Host ("CloudSimWeb build FAILED ({0})" -f $cfg) -ForegroundColor Red
        exit $LASTEXITCODE
    }
}

function Invoke-ConfigTests([string]$cfg) {
    Write-Host ''
    Write-Host ("======== Config: {0} ========" -f $cfg) -ForegroundColor Cyan
    Write-Host ("Bin: {0}" -f (Get-BinDir $cfg))

    Ensure-Binaries $cfg

    Write-Host ''
    Write-Host ("  SelfTest gate ({0}) ..." -f $cfg) -ForegroundColor Yellow
    & (Join-Path $CloudSimRoot 'scripts\run_selftest.ps1') -Configuration $cfg -Preset gate
    if ($LASTEXITCODE -ne 0) {
        Write-Host ("SelfTest FAILED ({0}) exit={1}" -f $cfg, $LASTEXITCODE) -ForegroundColor Red
        exit $LASTEXITCODE
    }

    Write-Host ''
    Write-Host ("  API pytest ({0}) ..." -f $cfg) -ForegroundColor Yellow
    & (Join-Path $CloudSimRoot 'scripts\run_api_tests.ps1') -Configuration $cfg
    if ($LASTEXITCODE -ne 0) {
        Write-Host ("API tests FAILED ({0}) exit={1}" -f $cfg, $LASTEXITCODE) -ForegroundColor Red
        exit $LASTEXITCODE
    }

    if ($WithSoak) {
        Write-Host ''
        Write-Host ("  soak {0} rounds ({1}) ..." -f $SoakRounds, $cfg) -ForegroundColor Yellow
        & (Join-Path $CloudSimRoot 'scripts\run_soak.ps1') -Configuration $cfg -Rounds $SoakRounds
        if ($LASTEXITCODE -ne 0) {
            Write-Host ("Soak FAILED ({0}) exit={1}" -f $cfg, $LASTEXITCODE) -ForegroundColor Red
            exit $LASTEXITCODE
        }
    }
}

$configs = if ($Configuration -eq 'Both') { @('Debug', 'Release') } else { @($Configuration) }

Write-Host ''
Write-Host '======== CloudSim one-click tests ========' -ForegroundColor Cyan
Write-Host ("Configs: {0}" -f ($configs -join ', '))
if ($FullBuild) { Write-Host 'FullBuild: yes' }

foreach ($cfg in $configs) {
    Invoke-ConfigTests $cfg
}

Write-Host ''
Write-Host '======== ALL PASSED (Debug+Release if Both) ========' -ForegroundColor Green
exit 0
