# KinematicCore 模块开发文档

## 1. 模块定位

`KinematicCore` 提供与 UI/OSG 无关的**树形运动学图**、FK、几何雅可比与通用 DLS IK。URDF/外轴/自定义设备在 `RobotUrdf` / `RobotScene` 侧构图为 `KinematicGraph` 后调用本库。

| 依赖 | 无 Qt/OSG（仅 C++ 标准库） |
| x64 输出 | `KinematicCore.dll` |
| 导出 | `KINEMATIC_CORE_API` |

---

## 2. 核心类型

| 类型 | 说明 |
|------|------|
| `JointMotion1D` | 1-DOF 平移/旋转；`hasLimit` + `lower`/`upper`；`qScale`（prismatic 米→mm） |
| `KinematicJoint` | `parentToChildRest` + `JointTransformOrder`（URDF 为 `RestThenMotion`） |
| `KinematicGraph` | 树形 link/joint 列表；`dofCount()` / `validateTree()` |
| `IKinematicModel` | `forward(q)` → 各 link 4×4 列主序 |

---

## 3. 求解器 API

| 函数 | 说明 |
|------|------|
| `forwardKinematicsTree` | 根 `baseWorld` × link rest × 关节 motion |
| `computePositionJacobian` / `computePoseJacobian` | 几何雅可比；旋转关节轴/枢轴在 **parent×Rest** 关节系 |
| `solvePoseDampedLeastSquares` | 通用 6-DOF/3-DOF DLS；迭代内 `clampJointAnglesToGraphLimits` |

**注意**：URDF 生产 IK 在 `RobotUrdf::KinematicCoreUrdfIk` 使用 `computeLinkWorldMatrices` 作 FK 真值，避免矩阵直提四元数与 OSG 显示不一致。

---

## 4. 工程筛选器

| 筛选器 | 内容 |
|--------|------|
| `inc\Motion` / `source\Motion` | `JointMotion1D`、`Mat4Ops`、`JointMotionEval` |
| `inc\Graph` / `source\Graph` | `KinematicGraph`、`KinematicLink`、`KinematicJoint` |
| `inc\Solvers` / `source\Solvers` | FK、雅可比、`DlsPoseIk` |
| `inc\Model` / `source\Model` | `IKinematicModel`、`AxisDescriptor` |

---

## 5. 依赖关系

```
RobotUrdf (buildUrdfKinematicGraph, KinematicCoreUrdfIk)
    → KinematicCore
RobotScene (CompositeKinematicModel, UrdfRobotKinematicModelSink)
    → KinematicCore + RobotUrdf
CloudSimHost (RobotServiceAdapter apply)
    → RobotScene Registry apply
```
