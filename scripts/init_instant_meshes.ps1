# 将 wjakob/instant-meshes 克隆到 bin/SDK/instant-meshes 并检出固定 commit
$ErrorActionPreference = "Stop"
$repoRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$imDir = Join-Path $repoRoot "bin\SDK\instant-meshes"
$pinSha = "7b3160864a2e1025af498c84cfed91cbfb613698"

if (-not (Test-Path $imDir)) {
    git clone https://github.com/wjakob/instant-meshes.git $imDir
}

Push-Location $imDir
try {
    git fetch --depth 1 origin $pinSha 2>$null
    git checkout $pinSha
    git submodule update --init --recursive ext/dset ext/pss ext/pcg32 ext/tbb
    Set-Content -Path (Join-Path $imDir "CLOUDSIM_PINNED_COMMIT") -Value @(
        $pinSha
        "# wjakob/instant-meshes pinned for CloudSim InstantMeshesLib"
    )
    Write-Host "instant-meshes ready at $imDir ($pinSha)"
}
finally {
    Pop-Location
}

$tbbBuild = Join-Path $imDir "ext\tbb\build-vs2019-x64"
if (-not (Test-Path (Join-Path $tbbBuild "Release\tbb_static.lib"))) {
    cmake -S (Join-Path $imDir "ext\tbb") -B $tbbBuild -G "Visual Studio 16 2019" -A x64 `
        "-DCMAKE_POLICY_VERSION_MINIMUM=3.5" `
        -DTBB_BUILD_SHARED=OFF -DTBB_BUILD_STATIC=ON `
        -DTBB_BUILD_TBBMALLOC=OFF -DTBB_BUILD_TBBMALLOC_PROXY=OFF -DTBB_BUILD_TESTS=OFF
    $msb = & "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe" `
        -latest -requires Microsoft.Component.MSBuild `
        -find "MSBuild\**\Bin\MSBuild.exe" | Select-Object -First 1
    & $msb (Join-Path $tbbBuild "tbb_static.vcxproj") /p:Configuration=Release /p:Platform=x64 /m
    Write-Host "built tbb_static.lib"
}
