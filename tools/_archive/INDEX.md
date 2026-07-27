# tools/_archive

一次性维护脚本归档。日常请用 `scripts/generate_vcxproj_filters.py` 等正式脚本。

| 文件 | 原路径 | 原用途 | 为何归档 |
|------|--------|--------|----------|
| `gen_builtins_filters.py` | `tools/` | 仅 Builtins 写 `.vcxproj.filters` | 已被 `scripts/generate_vcxproj_filters.py` 取代 |
| `gen_trajectory_algorithm_filters.py` | `tools/` | 仅 TrajectoryAlgorithm 写 filters | 同上 |
| `gen_trajectory_op_modules.py` | `tools/` | 批量生成部分 Op 头/源骨架 | 迁移期一次性生成器，无现网文档入口 |
| `patch_trajectory_vcxproj.py` | `tools/` | 轨迹相关 vcxproj 补丁 | 无文档引用；一次性工程修补 |
| `update_vcxproj_paths.py` | `tools/` | 批量改 vcxproj 路径 | 目录迁移期一次性 |
| `update_robotscene_vcxproj.py` | `tools/` | RobotScene 工程项更新 | 同上 |
| `update_atomic_vcxproj.py` | `tools/` | atomic 相关工程更新 | 同上 |
| `update_builtins_vcxproj.py` | `tools/` | Builtins 工程更新 | 同上 |
| `patch_controller.py` | `src/UI/RobotWidget/tools/` | 硬编码路径改 Controller include | 一次性源码补丁；路径已过时 |

## 仍留在 `tools/` 的相关脚本（保留）

| 文件 | 原因 |
|------|------|
| `gen_atomic_ops.py` | `TrajectoryAlgorithmBuiltins/DEVELOPER_GUIDE`、`RobotScene/resource/trajectory/README` 仍引用 |
| `gen_trajectory_json.py` | 同上 |
| `embed_vcxproj_filters.py` 等 | 未纳入本次「patch/update_*_vcxproj」归档范围 |
