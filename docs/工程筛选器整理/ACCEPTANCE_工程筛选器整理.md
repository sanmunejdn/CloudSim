# ACCEPTANCE — 工程筛选器整理

## 执行记录

| 任务 | 状态 | 说明 |
|------|------|------|
| T0 生成器脚本 | ✅ | `CloudSim/tools/RegenerateProjectFilters.ps1` |
| T1 Builtins | ✅ | `ops\<Op>` 顶层 + `inc\Common` / `src\Common`（22 filters） |
| T2 GeometryAlgorithm | ✅ | 功能夹 + discretizers/detail/流水线子树（15 filters，无 Other） |
| T3 Widget / RobotWidget | ✅ | 功能夹 + External 子路径（qtpropertybrowser / Host 头） |
| T4 Data / Scene / Host / PCA | ✅ | 按 DESIGN 归类；Host External 按来源嵌套 |
| T5 缺失 + 小工程 | ✅ | 5 个新建 filters；SLN 39 工程均有 `.filters` |
| T6 Assess | ✅ | 本文件 + FINAL / TODO |

## 验收检查

| 标准 | 结果 |
|------|------|
| 大工程功能筛选器可见 | ✅ |
| 5 个缺失 filters 已补 | ✅ CollisionAlgorithm / RobotCommSDK / IndustrialCameraSDK / IndustrialCameraPlugin / ProcessFlowPlugin |
| 其余工程轻量细分 | ✅ |
| SLN 外 CloudSimPluginHost / HelloAiPlugin | ✅ Ai 细分 + PluginHost/Document；HelloAi → Plugin |
| vcxproj 项均有 Filter | ✅（由脚本从 vcxproj 生成） |
| 跨工程 → External | ✅，并按相对路径镜像子夹 |
| 未改 vcxproj / 未搬源文件 | ✅ |

## 抽查摘要

- **TrajectoryAlgorithmBuiltins**：20 个 op 独立筛选器
- **GeometryAlgorithm**：`src\MeshSurfaceReconstruction`、`src\TubularGrinding`、`discretizers` 已镜像
- **CloudSimHost**：本工程功能夹 + `External\UI\CloudSimPluginHost\...` / `External\UI\Widget\...`
- **ProcessFlowPlugin**：`inc\sim` / `src\sim` + UI / Plugin / SimController
- **IndustrialCameraSDK**：`calib` / `hik` / `mech` / `pose` / Core
