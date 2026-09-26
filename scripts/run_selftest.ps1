# 运行 SelfTestRunner（先补齐 Qt/OSG PATH）

param(
    [Parameter(Mandatory = $false)]
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Debug',

    [Parameter(Mandatory = $false)]
    [ValidateSet('gate', 'full')]
    [string]$Preset = 'gate'
)

$ErrorActionPreference = 'Stop'
$CloudSimRoot = Split-Path -Parent $PSScriptRoot
$bin = if ($Configuration -eq 'Debug') {
    Join-Path $CloudSimRoot '..\bin\x64d'
} else {
    Join-Path $CloudSimRoot '..\bin\x64'
}
$bin = [System.IO.Path]::GetFullPath($bin)

$pathParts = New-Object System.Collections.Generic.List[string]
$pathParts.Add($bin)

if ($env:QTDIR -and (Test-Path (Join-Path $env:QTDIR 'bin'))) {
    $pathParts.Add((Join-Path $env:QTDIR 'bin'))
}
$qtFallback = 'D:\Qt\Qt5.14.2\5.14.2\msvc2017_64\bin'
if (Test-Path $qtFallback) {
    $pathParts.Add($qtFallback)
}

$osg = [System.IO.Path]::GetFullPath((Join-Path $CloudSimRoot '..\bin\SDK\OSG3.6.5\bin'))
if (Test-Path $osg) {
    $pathParts.Add($osg)
}

$env:PATH = (($pathParts | Select-Object -Unique) -join ';') + ';' + $env:PATH

$exe = Join-Path $bin 'SelfTestRunner.exe'
if (-not (Test-Path $exe)) {
    Write-Error "missing $exe"
    exit 1
}

Push-Location $bin
try {
    & $exe "--preset=$Preset"
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}
