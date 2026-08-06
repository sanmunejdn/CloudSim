# ACCEPTANCE — 网页端坐标系对齐

| # | 验收项 | 结果 |
|---|--------|------|
| 1 | GET/PUT `/api/robot/frames` | **通过**（已实现并编入 Gateway） |
| 2 | capture-tool / capture-user / reset-tool | **通过** |
| 3 | GET overlays + 前端 RGB 轴 | **通过**（`frameOverlays` 组） |
| 4 | 面板 CRUD / 激活 / 显示 / debounce PUT | **通过**（`#robotFrame`） |
| 5 | 工程保存写入 `robotKinematicsInstances[].coordinateFrames` | **通过**（`mergeRobotKinematicsIntoProjectRoot`） |
| 6 | Active/几何变更同步 tool context | **通过**（`syncProgramToolContextAfterFrameChange`） |
| 7 | Debug\|x64 + Release\|x64 | **通过**（`CloudSimHost` → Gateway → `CloudSimWeb.exe`；web 已拷至 `bin\x64d\web` / `bin\x64\web`） |

## 手工点验（建议）

导入 URDF → 坐标系 Tab → 编辑位姿/激活/显示 → 场景轴 → 捕获/重置 → 保存再打开。
