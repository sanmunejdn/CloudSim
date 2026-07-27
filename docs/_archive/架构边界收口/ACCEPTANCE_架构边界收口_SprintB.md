# ACCEPTANCE：架构边界收口（Sprint B）

> 承接 Sprint A；见 [ACCEPTANCE_架构边界收口.md](ACCEPTANCE_架构边界收口.md)。

## 范围

| ID | 项 | 验收标准 | 状态 |
|----|----|----------|------|
| B1 | 基座位姿 Mat4 化 | `IRobotDocumentHost` / `IRobotUrdfImportContext` 的 `setRobotBasePlacementWorldForInstance` / `updateRobotLinkOuterBindFromWorld` 使用 `core::Mat4`，接口头无 `osg/Matrixd` | **通过** |
| B2 | Host 写 kinematics 解耦 | `ProjectPackageIo` 不再 include `IRobotDocumentHost` / `RobotProjectIoAdapter`；Widget 直调 `RobotProjectIo::writeRobotKinematics` | **通过** |
| B3 | 视口工具栏经 render() | `MainWindowSceneInteractionCoordinator`、`DocumentPage` 工具栏查找走 `render().widget()` | **通过** |

## 未纳入本 Sprint

- `IRobotOsgViewHost` / `DocumentPage` 元数据全面去 OSG
- Host 去掉 `RobotWidget.lib` 链接（OsgWidget 编译单元仍可能需要类型头）
- `RobotSimulationController` 下沉 Host

## 验证清单

- [x] `CloudSimHost` + `RobotWidget` + `Widget` x64 Debug（via `CloudSim.sln`）
- [ ] 打开含机器人工程：基座位姿恢复正确；保存再打开 `basePlacementWorld` 一致
- [ ] 视口左右面板按钮仍可切换 Dock

## 变更摘要

- 契约：机器人基座/连杆 bind 写接口改为 `cloudsim::core::Mat4`
- Host：`RobotProjectKinematicsRestore` 按 OSG ptr 同序填 Mat4；移除 `mergeRobotKinematicsIntoProjectRoot`
- Widget：工程保存直调 RobotWidget 适配器；场景接线去 `OsgWidget.h`
