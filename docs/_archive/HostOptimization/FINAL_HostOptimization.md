# FINAL — HostOptimization（路径 B）

## 交付摘要

在不拆 Plugin/Osg DLL 的前提下，完成接口全景、契约上提样板、Host 同 DLL 逻辑分层、DocumentHost 状态外提、Controller 安全域拆分；可选 Web H2 / AiHost 已评估并延期。

## 主要产物

| 路径 | 说明 |
|------|------|
| `docs/HostOptimization/*` | ALIGNMENT / CONSENSUS / INTERFACE_CATALOG / BACKEND 清单 / Headless 对齐 / 可选评估 / ACCEPTANCE |
| `IDataService::findByClassName` | Core 契约扩展 |
| `IRobotDocumentHost::documentData` | Robot UI 契约数据面 |
| `CloudSimHost/inc/{import,project,robot,headless,follow}` | 逻辑分层 + 根目录 shim |
| `DocumentProjectSidecar` / `DocumentFollowState` | DocumentHost 瘦身 |
| `RobotSimulationController_AxisTargets.cpp` | Controller 首个域拆分 |

## 构建

`CloudSimCore` / `CloudSimHost` / `RobotWidget` / `Widget`：**Debug|x64** 与 **Release|x64** 均通过。

## 后续

见 [TODO_HostOptimization.md](TODO_HostOptimization.md)。
