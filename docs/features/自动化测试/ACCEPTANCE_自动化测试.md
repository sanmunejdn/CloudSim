# ACCEPTANCE — 自动化测试

| ID | 标准 | 结果 | 备注 |
|----|------|------|------|
| A1 | SelfTestRunner Debug/Release 可运行 | 通过 | `--preset=gate` 双配置退出码 0；`--full` 含慢测/Debug VCG assert |
| A2 | pytest API ≈10 条 | 通过 | Debug：8 passed, 2 skipped（无 resource/models） |
| A3 | run_checks.ps1 | 已交付 | 脚本就绪；本地可用 `run_selftest` + `run_api_tests` 分步验证 |
| A4 | CrashDump + PDB | 通过 | CrashDump.h；CloudSim/CloudSimWeb 链 dbghelp；Release 已 GenerateDebugInformation |
| A5 | soak 脚本 | 已交付 | `run_soak.ps1`；默认 200 轮 |
| A6 | CI yml + runner 文档 | 已交付 | `.github/workflows/ci.yml` + CI_RUNNER_SETUP.md |
| A7 | failure bundle + BUGFIX_LOOP | 已交付 | collect_failure_bundle.py + BUGFIX_LOOP.md |

## 手工验证摘录（本机）

```text
.\scripts\run_selftest.ps1 -Configuration Debug -Preset gate   → exit 0
.\scripts\run_selftest.ps1 -Configuration Release -Preset gate → exit 0
.\scripts\run_api_tests.ps1 -Configuration Debug               → 8 passed, 2 skipped
```
