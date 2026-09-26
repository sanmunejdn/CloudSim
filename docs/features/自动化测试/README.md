# 自动化测试

编译门禁 → 数值自检 → API 集成测试 → 崩溃捕获 → soak → CI → 失败修复闭环。

| 文档 | 说明 |
|------|------|
| [ALIGNMENT_自动化测试.md](ALIGNMENT_自动化测试.md) | 需求对齐与边界 |
| [CONSENSUS_自动化测试.md](CONSENSUS_自动化测试.md) | 验收标准与约定冻结 |
| [DESIGN_自动化测试.md](DESIGN_自动化测试.md) | 架构与数据流 |
| [TASK_自动化测试.md](TASK_自动化测试.md) | 原子任务 |
| [CI_RUNNER_SETUP.md](CI_RUNNER_SETUP.md) | self-hosted runner 配置 |
| [BUGFIX_LOOP.md](BUGFIX_LOOP.md) | 失败 → Agent 修复闭环 |
| [ACCEPTANCE_自动化测试.md](ACCEPTANCE_自动化测试.md) | 验收清单 |
| [FINAL_自动化测试.md](FINAL_自动化测试.md) | 交付总结 |
| [TODO_自动化测试.md](TODO_自动化测试.md) | 待办与缺配置 |

## 一键命令（在 `CloudSim/` 根）

| 方式 | 命令 |
|------|------|
| **双击** | [`一键测试.cmd`](../../一键测试.cmd)（**默认 Debug+Release**：缺产物自动编译 → SelfTest gate → API） |
| PowerShell | `.\scripts\run_tests.ps1`（默认 Both） |
| 只跑一种配置 | `.\scripts\run_tests.ps1 -Configuration Debug` |
| 强制重编后再测 | `.\scripts\run_tests.ps1 -FullBuild` |
| 含 soak 冒烟 | `.\scripts\run_tests.ps1 -WithSoak -SoakRounds 20` |
| 全量门禁（双配置） | `.\scripts\run_checks.ps1` |
| 仅 soak | `.\scripts\run_soak.ps1 -Rounds 200` |