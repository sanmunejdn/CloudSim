# FINAL — 自动化测试

## 交付摘要

已落地「编译门禁 → 数值自检 → API 集成测试 → 崩溃 dump → soak → CI → 修复闭环」骨架。

### 新增/关键路径

| 项 | 路径 |
|----|------|
| SelfTestRunner | `src/App/SelfTestRunner/` |
| CrashDump | `src/App/CloudSimBootstrap/inc/CrashDump.h` |
| PointCloud 转发 | `GeometryServices` `SelfTest.h/.cpp` |
| API 测试 | `tests/api/` |
| soak | `tests/soak/soak_loop.py` |
| 脚本 | `scripts/run_checks.ps1` / `run_selftest.ps1` / `run_api_tests.ps1` / `run_soak.ps1` / `collect_failure_bundle.py` |
| CI | `.github/workflows/ci.yml` |
| 文档 | `docs/features/自动化测试/` |

### 设计取舍

1. **gate vs full**：默认 `--preset=gate` 跳过 GeometryAlgorithm / PointCloud（过慢）与 VcgAlgorithms（Debug 第三方 assert 会 abort）。`--preset=full` 跑全量。
2. **PointCloudAlgorithm** 为静态库：经 GeometryServices DLL 转发 `runPointCloudSelfTest`，避免 SelfTestRunner 直链 PCL。
3. **PATH**：Qt/OSG 须在进程启动前进入 PATH（`run_selftest.ps1` / conftest / soak）；`main` 内改 PATH 赶不上导入表加载。

### 验证状态

- SelfTestRunner gate：Debug + Release 通过
- API pytest Debug：8 passed / 2 skipped
- CloudSimWeb / CloudSim Debug：含 CrashDump 编译通过
