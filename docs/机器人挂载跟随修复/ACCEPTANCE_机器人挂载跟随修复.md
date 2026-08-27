# ACCEPTANCE_机器人挂载跟随修复

## 任务完成记录

| 原子任务 | 状态 | 交付物 |
|---|---|---|
| T1 接口虚函数 | 完成 | `RobotScene/inc/IRobotSimulationDocument.h` 新增 `noteRobotJointAnglesAppliedForInstance`（默认空实现，追加于类尾，ABI 仅增不改槽位） |
| T2 收口回写 | 完成 | `RobotSceneKinematics.cpp` per-link 分支与层级分支各一处；`UrdfRobotKinematicModelSink.cpp` per-link 分支一处（notify 前） |
| T3 DocumentHost 直写 | 完成 | `noteRobotLocalJointAnglesForSceneRoot` 改为直写 local，去除聚合切片与 importContext 依赖 |
| T4 实现/转发 | 完成 | `DocumentPage` override；`MainWindowRobotHost::DocumentHost` 包装转发；`HeadlessRobotContext` override（自有缓存 + host 缓存双写） |
| T5 事件纯化 | 完成 | `publishRobotKinematicsApplied` 仅发事件；`CustomDeviceHostOps.cpp` 两处冗余回写删除 |
| T6 双配置编译 | 完成 | 见下表 |

## 编译验证（0 error；警告均为存量 C4005/C4251/LNK4099）

| 工程 | Debug\|x64 | Release\|x64 |
|---|---|---|
| RobotScene | 通过 | 通过 |
| CloudSimHost | 通过 | 通过 |
| RobotWidget | 通过（随 Widget 依赖链） | 通过（随 Widget 依赖链） |
| Widget | 通过 | 通过 |
| CloudSimWebGateway（含 CloudSimHostHeadless） | 通过 | 通过 |
| CloudSim（桌面主程序） | 通过 | 通过 |

产物目录以各 vcxproj 为准（`bin\x64d\` / `bin\x64\`），未改动任何 OutDir。

## 整体验收检查

- [x] 需求实现：关节角真源统一至 FK 收口，播放路径每帧回写，挂载刷新读到最新值
- [x] 编译通过：依赖链双配置全部 0 error
- [x] 实现与设计文档一致（DESIGN 的写入点、实现侧、纯化项一一对应）
- [x] 无聚合语义残留调用方（全仓 grep 确认仅余直写语义调用）
- [ ] 功能验证（需用户场景实测，见下）

## 待用户场景实测项（编译无法覆盖的功能验收）

1. 挂载设备 + 机器人 Run：设备全程逐帧跟随法兰；
2. Run 期间叠加设备姿态运动（运动到此 / DI 触发）：平滑叠加；
3. 含 DeviceAxis 段与运动段交替的程序；
4. 停止后设备位置 == 运行末帧位置（无瞬移）；
5. 回归：手动轴控跟随、TCP 拖动示教、点击预览、工程保存/加载后挂载位姿、Headless（Web）挂载刷新。

## 覆盖性自查（所有 FK 应用路径 → 真源回写）

| 路径 | 收口 | 回写 |
|---|---|---|
| 手动轴控/预览/TCP 拖动/停止落姿 | adapter → `applyRobotArm` → `UrdfRobotKinematicModelSink::applyToSink` | ✓（sink per-link 分支） |
| 播放 Run（桌面 + Headless 共用 Executor） | `applyJointAnglesFromDocument` | ✓（per-link 分支） |
| 点击预览 / 工程加载 / URDF 导入 / 播放引擎 | `applyJointAnglesFromDocument` | ✓ |
| Composite（机器人+外部轴） | `CompositeKinematicModel::applyToSink` → 同一 sink | ✓ |
| Headless TCP 拖动 IK | `applyJointAnglesForInstance(hrc,...)` → `applyJointAnglesFromDocument` | ✓ |
| 基座 gizmo 拖动（改 P 不改 q） | `applyPerLinkRobotBasePlacement` | 不需（关节角未变） |
