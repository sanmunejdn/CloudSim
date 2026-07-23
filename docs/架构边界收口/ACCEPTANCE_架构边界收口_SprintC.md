# ACCEPTANCE：架构边界收口（Sprint C）

> 承接 Sprint A/B；见同目录其他 ACCEPTANCE。

## 范围

| ID | 项 | 验收标准 | 状态 |
|----|----|----------|------|
| C1 | RobotProgramStore 上提 | 源码在 `RobotScene`；导出 `ROBOT_SCENE_API`；Host 通过 RobotScene 使用 | **通过** |
| C2 | Host 去 RobotWidget.dll | `CloudSimHost.vcxproj` 无 `RobotWidget.lib`；dumpbin 无 RobotWidget 导入 | **通过** |
| C3 | Host 规划准备内联 | `RobotPlanInstruction` 不再调用 `RobotInstructionPlanning::*` | **通过** |
| C4 | IRobotOsgViewHost Mat4 | 世界矩阵 / TCP teach 接口无 `osg::Matrixd` | **通过** |
| C5 | RobotOsgUiTypes 真源 | `OsgWidgetCore/inc/RobotOsgUiTypes.h`；RobotWidget 转发 | **通过** |
| C6 | Mat4 列主序统一 | DocumentPage `mat4FromOsg`/`osgFromMat4` 与 IRenderView 同为列主序 | **通过** |

## 未纳入

- `RobotOsgUi` 结构体内仍含 osg 类型（叠加路径）
- `IRobotBackendPoseSink` 仍 osg
- `RobotSimulationController` 下沉 Host

## 验证清单

- [x] `RobotScene` + `CloudSimHost` + `RobotWidget` + `Widget` x64 Debug
- [x] Host DLL 不再导入 RobotWidget.dll
- [ ] 打开/保存含机器人工程：基座位姿与程序目录正常
- [ ] TCP 拖拽示教仍可用

## 变更摘要

- `RobotProgramStore`：RobotWidget → RobotScene
- Host：去掉对 RobotWidget 的链接与规划 helpers 依赖
- `IRobotOsgViewHost`：矩阵 API → `core::Mat4`；调用方经 `RobotSimulationMath::getBackendRootWorldMatrixOsg` 适配
- `RobotOsgUiTypes`：迁入 OsgWidgetCore
