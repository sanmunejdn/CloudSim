# TASK / ACCEPTANCE — HostOptimization（路径 B）

## Wave1 — 文档

| 项 | 状态 |
|----|------|
| ALIGNMENT / CONSENSUS | 完成 |
| INTERFACE_CATALOG | 完成 |
| BACKEND_CALLSITE_INVENTORY | 完成 |

## Wave2 — 契约上提 + Host 目录

| 项 | 状态 |
|----|------|
| `IDataService::findByClassName` | 完成 |
| `IRobotDocumentHost::documentData` | 完成 |
| Controller `refreshAxisControlTargets` 改走 `documentData` | 完成 |
| Host `inc|source` 按 import/project/robot/headless/follow 分层 + 扁平 shim | 完成 |
| Debug\|x64 + Release\|x64 编译 | 见构建记录 |

## Wave3 — Headless 对齐 + DocumentHost 瘦身

| 项 | 状态 |
|----|------|
| HEADLESS_OPS_ALIGNMENT | 完成 |
| `DocumentProjectSidecar` / `DocumentFollowState` | 完成 |
| 公开 API（`backendSourcePath` 等）保持转发 | 完成 |

## Wave4 — Controller 切片

| 项 | 状态 |
|----|------|
| 已有 `_robotComm.cpp` | 保留 |
| 新增 `_AxisTargets.cpp`（首个安全域拆分） | 完成 |
| 大块 TCP 匿名命名空间拆分 | **不做**（易破命名空间；后续按方法级迁出） |

## 可选

| 项 | 状态 |
|----|------|
| Web H2 / AiHost | 见 OPTIONAL_EVAL；延期 |

## 构建验收

对 `CloudSimCore` → `CloudSimHost` → `RobotWidget` → `Widget` 执行 Debug|x64 与 Release|x64：**已通过**（2026-08）。
