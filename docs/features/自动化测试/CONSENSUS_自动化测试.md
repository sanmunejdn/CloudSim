# CONSENSUS — 自动化测试

## 1. 需求与验收

| ID | 标准 |
|----|------|
| A1 | `SelfTestRunner.exe` Debug/Release 均可运行；退出码 = 失败模块数（0 通过） |
| A2 | `tests/api` pytest 约 10 条核心用例；无机器人资源时 skip 而非 fail |
| A3 | `scripts/run_checks.ps1` 单命令：双配置编译 + 自检 + API 测试；失败非零退出 |
| A4 | CloudSim / CloudSimWeb 崩溃写 `{exe}/crash/<utc>_<pid>.dmp`；Release 带 PDB |
| A5 | soak 默认 200 轮；内存单调增长超阈值判失败 |
| A6 | `.github/workflows/ci.yml`：push 跑门禁，nightly 跑 soak（self-hosted） |
| A7 | 失败产物可 `collect_failure_bundle.py` 打包；修复必须补回归用例 |

## 2. 技术约束

- 产物路径遵循 `Directory.Build.props`：`bin\x64d\` / `bin\x64\`
- API 断言字段对齐 [API_CONTRACT.md](../网页端/全量对等/API_CONTRACT.md)
- 端口：fixture 动态分配，避免与默认 8787 冲突
- CI runner：Windows + VS2022 + Qt + Python（见 CI_RUNNER_SETUP.md）

## 3. 模块落点

| 组件 | 路径 |
|------|------|
| SelfTestRunner | `src/App/SelfTestRunner/` |
| CrashDump | `src/App/CloudSimBootstrap/inc/CrashDump.h` |
| API 测试 | `tests/api/` |
| soak | `tests/soak/soak_loop.py` |
| 脚本 | `scripts/run_checks.ps1` 等 |
| CI | `.github/workflows/ci.yml` |
