# 运动副设计说明

**范围**：CloudSim 中 1-DOF 平移/旋转关节的统一建模、求值与 FK 落盘。与 UI/OSG 无关的核心在 `KinematicCore`；领域配置在 `RobotUrdf` / `RobotScene` 侧构图后调用。

**架构图**：[运动副架构图.html](./运动副架构图.html)（浏览器打开）

---

## 1. 设计目标

| 目标 | 做法 |
|------|------|
| 一套 FK 内核服务 URDF 臂、外轴、自定义设备 | `kinematic_core::KinematicGraph` + `forwardKinematicsTree` |
| 1-DOF 参数可序列化、可适配多来源 | `JointMotion1D` + `JointMotionAdapters` |
| 视觉与求解器矩阵布局分离 | `CustomDeviceMat4Layout` 桥接 OSG Backend ↔ KC 列主序 |
| 旋转中心可绑 Frame | `originMm` 由父连杆系下 Frame 原点烘焙 |

---

## 2. 分层架构（摘要）

```
领域配置 (URDF / CustomDevice / ExternalAxis)
    → GraphBuilder + JointMotionAdapters
    → KinematicCore: evaluateJointMotion1D + forwardKinematicsTree
    → CustomDeviceMat4Layout（仅自定义设备/Backend 写回路径）
    → Backend worldMatrix / IRobotBackendPoseSink
```

外轴在 **Registry 复合模型** 内走 KinematicCore；`RobotExternalAxes` 仍保留 **OSG 布局** 下的直连求值（平移 `[3,7,11]`），与 KC 共轭顺序不同，勿混用。

---

## 3. 核心类型

### 3.1 `JointMotion1D`

统一 1-DOF 运动副参数（`KinematicCore/inc/JointMotion1D.h`）：

| 字段 | 含义 |
|------|------|
| `motionType` | `Translate` / `Revolute` |
| `axis[3]` | 单位轴（父关节系） |
| `originMm[3]` | 旋转枢轴（仅 Revolute；父关节系，mm） |
| `lower` / `upper` / `hasLimit` / `home` | 限位与零位 |
| `qScale` | q 乘数（URDF prismatic 米→mm 时为 `1000`） |

### 3.2 `KinematicJoint`

| 字段 | 含义 |
|------|------|
| `parentToChildRest[16]` | 零位下父→子刚体变换（列主序） |
| `transformOrder` | `MotionThenRest`（默认，自定义设备）或 `RestThenMotion`（URDF） |
| `motion` | `JointMotion1D` |
| `qIndex` | 在 q 向量中的下标；`-1` 表示固定副 |

### 3.3 FK 组合

对关节 `j`，令 `motion = evaluateJointMotion1D(j.motion, qj)`：

| `transformOrder` | 子连杆世界位姿 |
|----------------|----------------|
| `MotionThenRest` | `childWorld = parentWorld × motion × rest` |
| `RestThenMotion` | `childWorld = parentWorld × rest × motion` |

实现：`TreeForwardKinematics.cpp` → `mulRest`。

根连杆：`linkWorld[root] = baseWorld × link.restInBase`，再 BFS 展开关节树。

---

## 4. 运动矩阵求值 `evaluateJointMotion1D`

入口：`JointMotionEval.cpp`。

### 4.1 平移

`motion = T(axis · qEff)`，平移写入 KC 列主序 `[12,13,14]`。

### 4.2 旋转（KinematicCore）

绕 `originMm` 处 `axis` 旋转 `qEff`：

- **布局**：列主序，平移在 `[12,13,14]`
- **共轭**：`T(+o) · R · T(-o)`（`o = originMm`）

> **易错点**：OSG/Backend 底行平移布局（`[3,7,11]`）下正确共轭为 `T(-o)·R·T(+o)`，见 `RobotExternalAxes.cpp`。两套布局 **共轭顺序相反**，不可把外轴公式照抄进 KC。

### 4.3 雅可比

`GeometricJacobian.cpp` 中旋转轴/枢轴取在 **parent×Rest**（`RestThenMotion`）或 **parent×Motion**（`MotionThenRest`）关节系，与 FK 顺序一致。

---

## 5. 三条接入路径

| 来源 | 构图 | `transformOrder` | 旋转中心 |
|------|------|------------------|----------|
| **URDF** | `UrdfRobotLoader::buildUrdfKinematicGraph` | `RestThenMotion` | 关节 origin 在 `parentToChildRest` |
| **自定义设备** | `CustomDeviceGraphBuilder::buildGraph` | 默认 `MotionThenRest` | `motionCenterFrameBackendId` → `bakeMotionCenterFrameToOriginMm` → `originMm` |
| **外轴（Registry）** | `ExternalAxisGraphBuilder` | 同 KC | `originMm` 来自外轴配置 |

适配器：`JointMotionAdapters::fromCustomDeviceAxisConfig` / `fromRobotExternalAxisConfig`。

---

## 6. 自定义设备：rest 与 Frame 烘焙

### 6.1 `parentToChildRest`

`computeParentToChildRestFromLinkRestPoses`：在 `q=0` 下，用各连杆 `restInDeviceW0` 与设备 `W0` 反解父→子 rest，保证零位几何与组装姿态一致。`buildGraph` 每次重建图时重算。

### 6.2 Frame → `originMm`

1. 用户为旋转副选择「旋转中心」Frame（或连杆几何坐标系）。
2. **组装/提交**时 `rebakeRotateJointOriginsFromFrames`：父连杆 FK（当前 `q`）→ Frame 原点变到父连杆局部 → 写入 `motion.originMm`。
3. **轴控运行时**默认**不再**每帧从场景 Frame 反烘焙 `originMm`（`ApplyQOptions.rebakeOriginsFromSceneFrames=false`），保证链式机构枢轴随父连杆刚体运动，而非钉在世界坐标。

相关：`CustomDeviceKinematics.cpp`、`CustomDeviceAssemblyCommit::refreshLinkRestPosesFromGeometry`。

### 6.3 旋转中心 Frame 视觉回写

`applyQ` 在 FK 写回连杆几何后调用 `syncMotionCenterFramesFromOrigins`：

```text
W_frame = W_parentLink(q) × T(originMm)    // 仅 FrameBackendData；安装坐标系走挂载契约另算
```

- 跳过 Link 几何 `geometryBackendId`（已由 `applyToSink` 更新）。
- 挂载启用时跳过 `mountFrameBackendId`（由 `syncMountFrameWorldToTcpAlign` 维护）。
- Host：`flushCustomDeviceMotionCenterFrameVisual` 刷 OSG 外层 PAT。

### 6.4 写回视觉（连杆）

`CustomDeviceKinematicModel::applyToSink`：KC FK → `kinematicCoreToBackendMat4` → `setWorldMatrix` / `IRobotBackendPoseSink`。

统一入口：`KinematicModelApply::applyCustomDevice` → `CustomDeviceKinematics::applyQ`。

---

## 7. 矩阵布局对照

| | KinematicCore | Backend / OSG |
|---|---------------|---------------|
| 平移索引 | `[12,13,14]` | `[3,7,11]` |
| 旋转绕枢轴（Revolute） | `T(+o)·R·T(-o)` | `T(-o)·R·T(+o)` |
| 转换 | `CustomDeviceMat4Layout::osgBackendToKinematicCore` / 反向 |

---

## 8. 相关源码

| 模块 | 路径 |
|------|------|
| 运动副参数 | `src/Contracts/KinematicCore/inc/JointMotion1D.h` |
| q → 4×4 | `src/Contracts/KinematicCore/source/JointMotionEval.cpp` |
| FK 树 | `src/Contracts/KinematicCore/source/TreeForwardKinematics.cpp` |
| 适配器 | `src/Robot/RobotScene/source/JointMotionAdapters.cpp` |
| 自定义设备构图 | `src/Robot/RobotScene/source/CustomDeviceGraphBuilder.cpp` |
| Frame 烘焙 / applyQ | `src/Robot/RobotScene/source/CustomDeviceKinematics.cpp` |
| Mat4 桥接 | `src/Robot/RobotScene/source/CustomDeviceMat4Layout.cpp` |
| OSG 外轴直连 | `src/Robot/RobotScene/source/RobotExternalAxes.cpp` |
| Registry 应用 | `src/Robot/RobotScene/source/KinematicModelApply.cpp` |
| 模块指南 | `src/Contracts/KinematicCore/DEVELOPER_GUIDE.md` |

---

## 9. 验收要点

- 旋转副绑 Frame 后，拖动 q 子连杆绕 Frame 公转（到 Frame 距离恒定）。
- URDF 臂 FK/IK 与改共轭前行为一致（`originMm` 通常为 0）。
- Debug 产物对 `bin/x64d`，Release 对 `bin/x64`；改 `KinematicCore` / `RobotScene` 须双配置编译。
