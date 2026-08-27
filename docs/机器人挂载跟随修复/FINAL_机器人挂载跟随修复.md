# FINAL_机器人挂载跟随修复

## 项目总结

### 问题

自定义设备挂载机器人法兰后，机器人程序播放（Run）期间设备不跟随（钉在起点），运动结束后瞬移到正确位置。设备自身轴运动正常。

### 根因

「机器人当前关节角」状态真源分裂：连杆世界矩阵所有 FK 路径都会写，但挂载设备 TCP 重算所依赖的关节角缓存（`DocumentHost::m_robotLocalJointQBySceneRoot`）只有手动路径（adapter → `publishRobotKinematicsApplied`）写入。播放路径每帧触发挂载刷新却读到陈旧缓存，算出恒定旧 TCP；停止时 `applyJointAnglesRad` 刷新缓存导致瞬移。

### 修复（方案一：doc 级关节角真源，FK 收口写入）

- `IRobotSimulationDocument` 新增 `noteRobotJointAnglesAppliedForInstance(instIdx, localQ)`（默认空实现，追加类尾，ABI 兼容）；
- RobotScene 两个 FK 收口点回写：`applyJointAnglesFromDocument`（per-link + 层级两分支）、`UrdfRobotKinematicModelSink::applyToSink`（per-link 分支）——覆盖手动/播放/预览/加载/导入/Composite/Headless 全部路径；
- 实现侧：`DocumentPage` 直写 DocumentHost 缓存；`MainWindowRobotHost::DocumentHost` 包装转发（播放路径经此对象，缺它修复不生效）；`HeadlessRobotContext` 双写自有缓存与 host 缓存（播放桥种子读后者）；
- `DocumentHost::noteRobotLocalJointAnglesForSceneRoot` 纯化为直写 local；
- `publishRobotKinematicsApplied` 纯化为仅发事件；删除 `CustomDeviceHostOps` 两处冗余回写。

### 改动文件（10 个）

| 文件 | 改动 |
|---|---|
| `RobotScene/inc/IRobotSimulationDocument.h` | +7 行（新虚函数） |
| `RobotScene/source/RobotSceneKinematics.cpp` | +2 行（两处回写） |
| `RobotScene/source/UrdfRobotKinematicModelSink.cpp` | +1 行 |
| `CloudSimHost/inc/DocumentHost.h` | 注释与参数语义 |
| `CloudSimHost/source/DocumentHost.cpp` | 直写实现（-22/+8 行） |
| `CloudSimHost/source/follow/DocumentHostEvents.cpp` | -1 行（移除缓存写入） |
| `CloudSimHost/source/io/CustomDeviceHostOps.cpp` | -2 行（冗余回写） |
| `CloudSimHost/inc/headless/HeadlessRobotContext.h` / `source/headless/HeadlessRobotContext.cpp` | override 双写 |
| `UI/Widget/inc/DocumentPage.h` / `source/DocumentPage.cpp` | override |
| `UI/Widget/source/MainWindowRobotHost.cpp` | 包装转发 |

### 质量评估

- 代码：净增约 20 行，删除约 25 行；无新抽象、无推测性设计；每处改动可追溯到根因；
- 架构：状态写入收敛到 FK 收口（与 notify 同一抽象层），符合「状态写入点 ≤ 状态种类」原则；RobotScene 未反向依赖上层；
- 风险：vtable 仅尾部追加，未重建的二进制调用旧虚函数槽位不受影响；回写幂等（QHash 覆盖）；
- 性能：播放每帧每实例一次 O(nj) QHash 写入，可忽略；
- 编译：6 工程 × 双配置全部 0 error。

### 已知保留项

- `WebGatewayApi.cpp:545-549` 显式 `recordJointAnglesForSceneRoot` 保留（幂等同值，防御性）；
- `resolveEffectiveDeviceW0` 挂载分支仍未实现（方案二已否决，注释声明与实现不一致属存量问题，未扩大范围）。

## 后续优化方向

方案三（统一运动时钟 / 场景运动调度器）已列入 `TODO_机器人挂载跟随修复.md`。
