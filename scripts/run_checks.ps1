# 一键门禁：双配置编译 + SelfTestRunner + API 测试

param(
    [Parameter(Mandatory = $false)]
    [switch]$SkipBuild,

    [Parameter(Mandatory = $false)]
    [switch]$SkipApi,

    [Parameter(Mandatory = $false)]
    [string]$MsBuild = ''
)

$ErrorActionPreference = 'Stop'
$CloudSimRoot = Split-Path -Parent $PSScriptRoot
Set-Location $CloudSimRoot

$stamp = Get-Date -Format 'yyyyMMdd_HHmmss'
$art = Join-Path $CloudSimRoot "artifacts\checks\$stamp"
New-Item -ItemType Directory -Force -Path $art | Out-Null
$summary = @()

function Find-MsBuild {
    if ($MsBuild -and (Test-Path $MsBuild)) { return $MsBuild }
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $p = & $vswhere -latest -requires Microsoft.Component.MSBuild -find 'MSBuild\**\Bin\MSBuild.exe' | Select-Object -First 1
        if ($p) { return $p }
    }
    return 'msbuild'
}

function Invoke-Logged($name, $scriptBlock) {
    $log = Join-Path $art "$name.log"
    Write-Host "=== $name ==="
    try {
        & $scriptBlock *>&1 | Tee-Object -FilePath $log
        if ($LASTEXITCODE -ne 0 -and $null -ne $LASTEXITCODE) {
            $script:summary += "[FAIL] $name (exit=$LASTEXITCODE)"
            return $false
        }
        $script:summary += "[PASS] $name"
        return $true
    }
    catch {
        $_ | Out-File -FilePath $log -Append
        $script:summary += "[FAIL] $name ($_)"
        return $false
    }
}

$failed = $false
$msb = Find-MsBuild
$sln = Join-Path $CloudSimRoot 'CloudSim.sln'
$runnerProj = Join-Path $CloudSimRoot 'src\App\SelfTestRunner\SelfTestRunner.vcxproj'
$webProj = Join-Path $CloudSimRoot 'src\App\CloudSimWeb\CloudSimWeb.vcxproj'

if (-not $SkipBuild) {
    foreach ($cfg in @('Debug', 'Release')) {
        $ok = Invoke-Logged "build-SelfTestRunner-$cfg" {
            & $msb $runnerProj /p:Configuration=$cfg /p:Platform=x64 /m
            if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
        }
        if (-not $ok) { $failed = $true }

        $ok = Invoke-Logged "build-CloudSimWeb-$cfg" {
            & $msb $webProj /p:Configuration=$cfg /p:Platform=x64 /m
            if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
        }
        if (-not $ok) { $failed = $true }
    }
}

foreach ($cfg in @('Debug', 'Release')) {
    $ok = Invoke-Logged "selftest-$cfg" {
        & (Join-Path $CloudSimRoot 'scripts\run_selftest.ps1') -Configuration $cfg -Preset gate
        if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    }
    if (-not $ok) { $failed = $true }
}

if (-not $SkipApi) {
    foreach ($cfg in @('Debug', 'Release')) {
        $ok = Invoke-Logged "api-$cfg" {
            & (Join-Path $CloudSimRoot 'scripts\run_api_tests.ps1') -Configuration $cfg
            if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
        }
        if (-not $ok) { $failed = $true }
    }
}

$summaryPath = Join-Path $art 'SUMMARY.txt'
$summary | Out-File -FilePath $summaryPath -Encoding utf8
Write-Host ''
Write-Host '==== SUMMARY ===='
$summary | ForEach-Object { Write-Host $_ }
Write-Host "artifacts: $art"

if ($failed) { exit 1 }
exit 0
