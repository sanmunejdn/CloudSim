# DESIGN — Robot 运动学演进

## 目标架构

见 [diagrams/target-architecture.html](diagrams/target-architecture.html)。

| 层 | 职责 |
|----|------|
| RobotWidget | 拖动节流、chase、写回场景 |
| RobotScene TeachIk | 外轴 bake/unbake、联立多候选代价 |
| RobotUrdf | 模型缓存、索引图、FK/J、Workspace、UrdfNumericalIk、Options |
| RobotKinematics | CircularArcGeometry；SerialLinkKinematics = legacy |

拖动热路径见 [diagrams/drag-hotpath-dataflow.html](diagrams/drag-hotpath-dataflow.html)。

## Workspace

`UrdfKinematicsWorkspace`：预分配 `qRad`、`J`、`cols`、BFS 队列（linkId int）。  
`UrdfFkModelData` 增补 `linkNames`、`linkNameToIndex`、`childJointIndices`（按父 link 索引）。

热路径 API：`computeLinkPoseAndGeometricJacobian(..., UrdfKinematicsWorkspace&)`；旧签名转发到 thread_local workspace。

## Options

```text
TeachIkSolverOptions / UrdfIkSolverOptions
  lambda=1e-2
  positionToleranceMm=1e-2
  orientationToleranceRad≈0.1°
  orientationWeight=300
  maxJointStepRad（0=沿用内部 0.2）
  maxIterations（0→180）
```

## filters（RobotUrdf）

| Filter | 内容 |
|--------|------|
| Global | robot_urdf_global.h |
| Loader | 解析/导出/场景构建 |
| Kinematics | Workspace、NumericalIk、Options |
| SelfTest | SelfTest + golden |

## Oracle

SelfTest：最小 fixture URDF → FK golden JSON、几何 J vs 有限差分、DLS 闭环。  
`tools/pinocchio_oracle/generate_goldens.py` 离线可选重生。
