# ACCEPTANCE：架构边界收口（Sprint F）

> 日期：2026-07-21  
> 范围：`IRobotSimulationDocument` 去 osg；FK/关节契约改 Core `Mat4` / DTO

## 验收项

| ID | 项 | 证据 | 结果 |
|----|----|------|------|
| F1 | 仿真文档接口无 osg | `IRobotSimulationDocument.h` 无 osg include | **通过** |
| F2 | per-link 切片 DTO | `robotPerLinkKinematicsForInstance(..., RobotPerLinkKinematicsSliceDto&)` | **通过** |
| F3 | FK 表 Mat4 | `robotFkMeshWorldT0` / `robotOuterWorldAtBind` 返回 `QHash<QString, Mat4>` | **通过** |
| F4 | 关节无裸 MT 指针 | `hasRobotJointLocalMatrix` / `robotJointWorldMatrix` / `applyRobotJointLocalMatrix(ices)` | **通过** |
| F5 | OSG 切片隔离 | `RobotPerLinkKinematicsSliceOsg.h` + `robotPerLinkSliceFromDto`；RobotScene/Host 内部转换 | **通过** |
| F6 | 编译 | `CloudSim.sln` `/t:RobotScene;CloudSimHost;RobotWidget;Widget` Debug\|x64 | **通过** |

## 已知残余

- `DocumentPage` 内部 `HierarchicalRobotInstance` 仍存 `osg::Matrixd` / `MatrixTransform*`（实现细节）
- `DocumentPage::backend()` / `BackendDataManager` 穿透待定
- `IRobotBackendPoseSink` 仍含 osg

## 手工回归建议

- per-link / 层级机器人关节驱动与回放
- 基座 gizmo 拖动后 FK 刷新
- 工程保存/加载 `basePlacementWorld`
- TCP 示教（层级末关节世界矩阵路径）

## 变更摘要

- `IRobotSimulationDocument`：去掉 osg 类型与 `robotJointMatrixTransform`
- OSG 形态切片迁入 `RobotPerLinkKinematicsSliceOsg.h`
- 调用方经 DTO 取数，需要时再 `robotPerLinkSliceFromDto`
