# 07 — URDF 与机器人场景

实现真源：[RobotUrdf](../../src/Robot/RobotUrdf/DEVELOPER_GUIDE.md) · [RobotScene](../../src/Robot/RobotScene/DEVELOPER_GUIDE.md) · [spatial_contract](../spatial_contract_world_pose.md) § URDF。

## 两种场景路径

| 路径 | 行为 | 用途 |
|------|------|------|
| 层级装配树 `buildHierarchicalRobotScene` | 关节 `MatrixTransform` + mesh | 直接挂整树、关节角写 joint MT |
| **每连杆后端（主推）** | 每 visual link → `MeshBackendData` + OSG 分支；FK 写 outer `worldMatrix` | 属性编辑、工程序列化、`robotKinematicsInstances` |

开关：`UrdfRobotLoader.cpp` 内 `kUseMeshBackendForLinks`（默认 `true`）；失败回退 `osgDB`。管理层：`UrdfLinkBackendManager`。

## 长度与矩阵

- 内部长度统一 **mm**（URDF `<origin>` 从米转换）
- 显示侧经适配器保证与 **行向量右乘** 语义一致
- 权威世界位姿仍是 `worldMatrix`；连杆 geometry 保持 link 文件系
- **禁止**把 FK 再烘焙进顶点后又写一份姿态

## FK / IK

见 RobotUrdf 指南：`computeLinkWorldMatrices` / `RigidTransforms`、`solveArmPoseDampedLeastSquares`（主路径经 KinematicCore）。

指令与轨迹：[指令IK轨迹](../features/指令IK轨迹/) · 路径规划：[机器人路径规划](../features/机器人路径规划/) · 运动副：[运动副](../features/运动副/)。

## Mesh 后端化

| 能力 | osgDB 直读 | MeshBackend |
|------|------------|-------------|
| 加载 | 通用插件 | CGAL/OCC 等专用解析，通常更快 |
| 属性编辑 | 弱 | 颜色等可进后端 |
| 工程保存 | 几何易丢 | 可随后端序列化 |
| 管理 | 散落 OSG 节点 | 注册表 / Units 树统一 |

A/B：将 `kUseMeshBackendForLinks` 设为 `false` 后重编 RobotUrdf（及依赖），判断显示异常是否出在后端路径。

## 显示异常（长条状 / 炸开）

1. 先 A/B 开关后端路径，区分「后端加载」vs「通用 FK/挂载」
2. 看加载日志：三角形数、耗时；对角缩放≈1，平移为合理 mm
3. 核对 `meshToLink` / visual origin：后端常把 visual origin **烘焙进顶点**，pose 侧勿二次变换
4. 对照 spatial_contract：禁止旧式独立 `pose`/`rotation` JSON 与手写 `T×R`

## 关节轴位置

- BFS 挂载日志：parent/child link 与 URDF 一致
- `T_origin` 平移 = URDF origin（米→毫米后）
- 轴显示错位但 mesh 正确 → 优先查轴 geode / 挂载父节点

## 自定义设备 / 挂载

见 [RobotWidget](../../src/UI/RobotWidget/DEVELOPER_GUIDE.md) · [CloudSimHost](../../src/Host/CloudSimHost/DEVELOPER_GUIDE.md)。

---

← [06 模式与插件](06-模式与插件.md) · [08 网页端 →](08-网页端.md)
