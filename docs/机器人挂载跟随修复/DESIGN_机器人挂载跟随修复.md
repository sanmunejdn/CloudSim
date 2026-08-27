# DESIGN_机器人挂载跟随修复

## 整体架构

```mermaid
flowchart TD
  subgraph 写入路径["FK 应用路径（全部收口）"]
    A1["手动：RobotServiceAdapter::applyJointAnglesRad"]
    A2["播放：RobotProgramExecutor::tick"]
    A3["其它：预览 / 工程加载 / URDF 导入 / 播放引擎"]
    A1 --> B1["KinematicModelApply::applyRobotArm"]
    B1 --> B2["UrdfRobotKinematicModelSink::applyToSink<br/>（Composite 汇入同一 sink）"]
    A2 --> B3["RobotSceneKinematics::applyJointAnglesFromDocument"]
    A3 --> B3
    B2 -.非 per-link 回退.-> B3
  end

  subgraph 收口["RobotScene 收口点（新增回写）"]
    B2 --> C["doc->noteRobotJointAnglesAppliedForInstance(instIdx, localQ)"]
    B3 --> C
    C --> D["doc->notifyRobotKinematicsAppliedToScene()"]
  end

  subgraph 实现侧["IRobotSimulationDocument 实现"]
    C -.-> E1["DocumentPage<br/>→ DocumentHost::noteRobotLocalJointAnglesForSceneRoot"]
    C -.-> E2["MainWindowRobotHost::DocumentHost<br/>→ 转发 m_page"]
    C -.-> E3["HeadlessRobotContext<br/>→ recordJointAnglesForSceneRoot + host 缓存"]
  end

  D --> F["runBackendFollowSolveAndSync"]
  F --> G["refreshCustomDevicesFollowingKinematicsTargets"]
  G --> H["updateMountedDeviceWorldFromRobotTcp<br/>tryResolveMountFlangeTcpFromUrdfFk<br/>读取关节角真源（恒新）"]
  E1 -.真源.-> H
  E3 -.真源.-> H
```

## 分层设计

| 层 | 模块 | 职责 |
|---|---|---|
| 接口 | RobotScene `IRobotSimulationDocument` | 新增回写虚函数（默认空实现，不强制实现方） |
| 收口 | RobotScene `RobotSceneKinematics` / `UrdfRobotKinematicModelSink` | FK 应用成功当场回写本实例关节角 |
| 状态 | CloudSimHost `DocumentHost` | `m_robotLocalJointQBySceneRoot` 真源；`noteRobotLocalJointAnglesForSceneRoot` 直写 local |
| 桌面实现 | Widget `DocumentPage` | instIdx → sceneRoot → 写真源 |
| 适配 | Widget `MainWindowRobotHost::DocumentHost` | 包装转发（播放路径经此对象） |
| Headless | CloudSimHost `HeadlessRobotContext` | 双写自有缓存 + host 缓存 |
| 事件 | CloudSimHost `DocumentHostEvents` | `publishRobotKinematicsApplied` 纯化为事件发布 |

## 接口契约

```cpp
// RobotScene/inc/IRobotSimulationDocument.h
/// FK 收口回写：本实例当前关节角（rad）写入文档真源，供挂载设备 TCP 重算等读取
virtual void noteRobotJointAnglesAppliedForInstance(int instanceIndex, const QVector<double>& localJointRad);
```

| 契约项 | 约定 |
|---|---|
| 调用时机 | 仅在 FK 成功写入场景后、notify 前/伴随调用 |
| 参数 | `instanceIndex` 为机器人实例下标；`localJointRad` 为本实例局部关节角（rad，非聚合向量） |
| 线程 | 与现有 FK 应用同线程（UI 线程 / Headless 播放定时器线程） |
| 默认实现 | 空（不强制；mock/未来实现可忽略） |
| 幂等 | 同值重复写入无副作用（QHash insert 覆盖） |

## 数据流（修复后播放路径）

```mermaid
sequenceDiagram
  participant T as 播放定时器
  participant Ex as RobotProgramExecutor
  participant RK as RobotSceneKinematics
  participant Doc as IRobotSimulationDocument
  participant Host as DocumentHost 真源
  participant Mount as 挂载刷新

  T->>Ex: tick(doc, sink)
  Ex->>RK: applyJointAnglesFromDocument(doc, osg, q)
  RK->>RK: per-link FK 写连杆 Data/OSG
  RK->>Doc: noteRobotJointAnglesAppliedForInstance(i, jointSlice)
  Doc->>Host: 写 m_robotLocalJointQBySceneRoot（最新 q）
  RK->>Doc: notifyRobotKinematicsAppliedToScene()
  Doc->>Mount: runFollowSolveAndSync + refreshCustomDevicesFollowingKinematicsTargets
  Mount->>Host: robotLocalJointAnglesForSceneRoot（读到最新 q）
  Mount->>Mount: URDF FK → TCP → device.setWorldMatrix + applyQ
  Note over Mount: 设备逐帧跟随法兰，停止时无瞬移
```

## 异常处理策略

| 场景 | 行为 |
|---|---|
| instIdx 越界 / sceneRoot 为空 | 实现侧静默 return（与现有防御风格一致） |
| localJointRad 为空 | `noteRobotLocalJointAnglesForSceneRoot` 静默 return |
| FK 失败 | 不回写（保持上一次真值），沿现有 `return false` 路径 |
| legacy 回退路径（nInst==0） | 无实例概念，不回写（挂载依赖 per-link 绑定，该路径无消费者） |
| 多实例 | 每实例独立回写；挂载刷新按 `mount->robotSceneBackendId` 查对应实例缓存 |

## 性能评估

播放每帧每实例一次 `QHash::insert`（nj≤7 的 `QVector<double>` 拷贝），O(nj)，相对每帧 FK + follow 求解开销可忽略。
