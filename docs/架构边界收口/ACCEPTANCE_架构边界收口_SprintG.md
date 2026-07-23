# ACCEPTANCE：架构边界收口（Sprint G）

> 日期：2026-07-21  
> 范围：`IRobotBackendPoseSink` 去 osg；世界矩阵改 Core 列主序 `Mat4`

## 验收项

| ID | 项 | 证据 | 结果 |
|----|----|------|------|
| G1 | PoseSink 接口无 osg | `IRobotBackendPoseSink.h` 仅 `CoreTypes.h` | **通过** |
| G2 | get/set 用 Mat4 | `getBackendRootWorldMatrix` / `setBackendRootWorldMatrixFromWorld` | **通过** |
| G3 | OsgWidget 双形态 | osg::Matrixd 重载供内部；Mat4 override 实现契约 | **通过** |
| G4 | RobotScene 边界转换 | `RobotSceneKinematics` 经 `get/setBackendRootWorldOsg` 适配 | **通过** |
| G5 | 编译 | `CloudSim.sln` `/t:RobotScene;CloudSimHost;RobotWidget;Widget` Debug\|x64 | **通过** |

## 顺带修复

- `PerLinkKinematicsHostImpl` 误用的 `setBackendRootWorldMatrix` → `setBackendRootWorldMatrixFromWorld`（Mat4）

## 已知残余

- `DocumentPage::backend()` / `BackendDataManager` 穿透仍在
- `DocumentPage` 内部 `HierarchicalRobotInstance` 仍存 osg
- `OsgWidget` 内部与部分 Host Follow 路径仍直接用 osg 重载（允许）

## 手工回归建议

- per-link FK / 回放写位姿
- URDF 导入后连杆世界矩阵
- 基座 gizmo / Follow 附着（经 OsgWidget 重载）

## 变更摘要

- `IRobotBackendPoseSink`：`osg::Matrixd` → `cloudsim::core::Mat4`
- `OsgWidget`：保留 osg 重载 + 新增 Mat4 契约实现
