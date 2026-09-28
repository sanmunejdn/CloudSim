# 源码/架构巡逻单入口（P2-⑩）
$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
Set-Location $Root

$failed = 0
function Run-Step([string]$Name, [scriptblock]$Block) {
    Write-Host "==> $Name"
    & $Block
    if ($LASTEXITCODE -ne 0) { Write-Host "FAIL $Name"; $script:failed++ } else { Write-Host "OK $Name" }
}

Run-Step "host-headless" { python scripts/check_host_headless_sources.py }
Run-Step "backend-callsites" { python scripts/check_backend_callsites.py }
Run-Step "plugin-iid" { python scripts/check_plugin_iid_alignment.py }
Run-Step "filters-host" { python scripts/generate_vcxproj_filters.py --sync --project CloudSimHost }
Run-Step "clang-format" { python scripts/run_clang_format.py --check }
Run-Step "encoding" { python scripts/normalize_source_encoding.py --check }
Run-Step "header-guards" { python scripts/normalize_header_guards.py --check }

if ($failed -gt 0) { exit 1 }
Write-Host "All source checks passed."
exit 0
