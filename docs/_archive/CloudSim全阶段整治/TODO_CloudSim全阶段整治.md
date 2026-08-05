# TODO — CloudSim 全阶段整治（剩余）

## 手测（建议）

1. 轨迹编辑：调色板拖入、列表内重排（含落回原位）、Preview/Apply 后列表与块数一致
2. 外轴启用：联立无解时应规划失败（非静默臂-only）；有示教外轴值的指令在无外轴 plan 时字段仍保留
3. 坏 JSON 工程：打开时 RunLogger 出现 robotCollision / coordinateFrames / externalAxes 警告

## 审计延后项（本轮不改）

| ID | 摘要 |
|----|------|
| R3 | LINE+外轴多样本只保留终点 qe |
| R4 | 示教路径 `jointTrajectoryRad.clear()` |
| R5 | Executor 维数不匹配仍推进段 |
| R6 | 万级 pose axes 无 stride |
| R7 | 计划无外轴目标时滑轨停旧值 |
| C05–C11 | 非关键空 catch |

## 可选卫生

- [x] 无文档引用的一次性 `tools/gen_*.py` 已迁入 `tools/_archive/`（见该目录 INDEX；`gen_atomic_ops.py` / `gen_trajectory_json.py` 保留）
- [x] `tools/patch_*.py`、`tools/update_*_vcxproj.py` 及 `RobotWidget/tools/patch_controller.py` 已迁入 `tools/_archive/`
- 本地 `.vs/` 体积清理（gitignore，不入库）

## 配置/环境

- 完整 GUI 手测需本机 Qt/OSG/OCCT SDK 与 `bin\x64d` 工作目录
- 未编 `CloudSim.exe` 全链（Widget 及依赖已双编通过）；若需整包可再编 App 工程
