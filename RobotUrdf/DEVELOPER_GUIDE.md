# RobotUrdf 模块开发文档

## 1. 模块定位

`RobotUrdf` 负责 **URDF 解析**、**OSG 层级机器人场景**构建、**每连杆 Mesh 后端**批量创建，以及 FK 所需的 link/joint 元数据。长度内部统一 **mm**（URDF origin 由米转换）。

| 依赖 | `Data`, `RunLogger`；可选 `BackendVisual`（每连杆） |
| 导出 | `ROBOT_URDF_API` |

---

## 2. `namespace UrdfRobotLoader` — 元数据 API

| 函数 | 返回值 / 作用 |
|------|----------------|
| `loadRevoluteJointNamesInOrder(urdfPath)` | BFS 顺序关节名（与 `jointAnglesRad` 下标一致） |
| `loadRevoluteJointMeta` | 同上 + `lower`/`upper`（rad） |
| `loadRevoluteJointChildLinksInOrder` | 子 link 名列表 |
| `loadPrimaryTerminalLinkName` | 最深叶 link（TCP 候选） |
| `loadLinkChildToParentMap` | child link → parent link |
| `enumerateLinkVisualMeshes` | link → 绝对 mesh 路径 |
| `clearUrdfModelCache()` | 释放解析缓存 |

---

## 3. 运动学 / 矩阵 API

| 函数 | 说明 |
|------|------|
| `computeJointTransformMatrices(urdf, angles, outMap, err)` | 关节名 → 变换；旋转关节仅 R(q) |
| `computeLinkWorldMatrices(urdf, angles, out, err)` | 各 link **坐标系**世界矩阵（OSG） |
| `computeLinkWorldRigidTransforms(urdf, angles, out, err)` | 同上，输出 `engine::RigidTransform`（推荐机器人/IK 边界） |
| `computeMeshWorldMatrices(urdf, angles, out, err, meshVerticesAlreadyInLinkFrame)` | 各 link **网格**世界矩阵；末参 `true` 时 visual 为单位（顶点已在连杆系） |
| `linkMeshFileToLinkColumnMajor16(urdf, linkName, out16, err)` | mesh 文件系 → 连杆系 4×4（列主序），供导入烘焙顶点 |

---

## 4. 场景构建

### 4.1 `buildHierarchicalRobotScene`

| 项 | 说明 |
|----|------|
| 输出 | `osg::Group*`（调用方持有 ref_count） |
| 结构 | 关节 `MatrixTransform` + 几何 `MatrixTransform(meshToLink)` → mesh |
| 特点 | **无** `BackendVisual` 内层 `-bboxCenter` PAT |
| 用途 | `OsgWidget::addHierarchicalRobotScene`；关节角直接写各 `joint_*` MT |

### 4.2 每连杆后端路径（主推）

不挂整棵装配树；每个带 visual 的 link → 独立 `MeshBackendData` + OSG 分支，FK 由 `RobotSceneKinematics::applyJointAnglesViaLinkBackends` 写各 link **outer 世界矩阵**。

---

## 5. `class UrdfLinkBackendManager`

**目的**：一 link 一 `MeshBackendData`，便于属性编辑与 `project.json` 的 `robotKinematicsInstances`。

| 方法 | 说明 |
|------|------|
| `createLinkBackend(linkName, meshPath, visualOriginMatrix[16], err)` | 创建或返回已有 |
| `createLinkVisualNode(linkName, options, err)` | `BackendVisualRegistry` 建节点 |
| `getLinkBackend` / `getLinkBackendId` | 查询 |
| `batchCreateLinkBackends(linkMeshPaths, robotName, err)` | 批量 |
| `batchCreateVisualNodes(linkContainers, options, err)` | 挂到容器 MT |
| `setRobotName` / `robotName()` | backend id 前缀 |
| `setUseBackendLoading` / `useBackendLoading()`（静态） | A/B：`BackendVisual` vs `osgDB` |
| `getStats()` | `totalBackends`, `totalTriangleCount`, `avgLoadTimeMs` |
| `clear()` | 清空 |

---

## 6. `RobotSimulationTypes.h`

### `struct RobotSimulationCommand`（遗留简单回放）

| 字段 | 默认 | 说明 |
|------|------|------|
| `jointIndex` | 0 | 文档旋转关节列表下标 |
| `angleDeg` | 45 | 目标角（度） |
| `durationSec` | 2 | 段时长 |

### `namespace RobotSimulation`

| 常量 | 值 |
|------|-----|
| `kPi` | π |
| `kPlaybackTimerIntervalMs` | 16（与 MainWindow 定时器一致） |

---

## 7. `RobotAxisControlWidget`（`UrdfRobotLoader` 内嵌）

| 方法 | 说明 |
|------|------|
| `setupJointControls(names, limits, jointTransforms)` | 滑块 + OSG `MatrixTransform` 绑定 |
| `jointAnglesRad()` / `setJointAnglesRad` | 批量角 |
| `setJointAngle(name, rad)` | 单关节 |

**信号**：`jointAngleChanged`, `allJointAnglesChanged`。

---

## 8. 多机与命名

- 每台机器人独立 **`jointKeyPrefix`**（如 `RobotURDF_M-20iD-35::`）。
- `DocumentPage::appendHierarchicalRobotSimulationContext` **追加**实例，二次导入不清空。
- robot root id：`RobotURDF_<模型基名>`；link id：`rootId + "_" + linkName`。

---

## 9. 层级 vs 每连杆（约定一致性）

| 路径 | 位姿表达 |
|------|----------|
| 层级 | 关节 MT + 几何 MT |
| 每连杆 | `BackendDataManager` 父子边 + outer 世界矩阵 |

**必须一致**：mesh 文件系 / 连杆系 / 世界系；否则会出现「矩阵日志正确但模型散开」。导入时：`linkMeshFileToLinkColumnMajor16` + `transformVertices` + `skipInnerModelCenterRebase`。

---

## 10. 与坐标系 / TCP

- 默认终端 link：`loadPrimaryTerminalLinkName` / 最深子 link，供 `makeDefaultFrameSet(flangeLinkName)` 与示教捕获。
- FK：`computeLinkWorldMatrices` 提供 `T_base_flange`；工具偏移 `T_flange_tool` 在 `RobotScene::RobotCoordinate` 中与场景矩阵同一 OSG 行向量约定。

## 11. 相关文档

- 场景 FK 写回：[`../RobotScene/DEVELOPER_GUIDE.md`](../RobotScene/DEVELOPER_GUIDE.md)
- UI 导入：[`../Widget/DEVELOPER_GUIDE.md`](../Widget/DEVELOPER_GUIDE.md) §`registerUrdfRobot`
- 架构 §6.1：[`../ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md)
