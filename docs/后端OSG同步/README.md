# 后端 → OSG 单轨同步引擎

对齐 [`spatial_contract_world_pose.md`](../../docs/spatial_contract_world_pose.md) v2。

## 写路径（唯一真源）

`BackendDataBase::worldMatrix` ← FK / 属性 / Follow / gizmo 提交

## 读路径（唯一派生）

`BackendVisualSyncEngine::flushTransform` → `OsgWidget::applyWorldMatrixToOsg`

公式（行向量 OSG）：`local = worldMatrix × inv(parentWorld)`；逻辑父 `parentWorld` 优先读 **Data**（`BackendDataManager::parentsOf` + `m_backendParentIds`）。

## 核心类型

| 类型 | 路径 |
|------|------|
| `BackendVisualSyncEngine` | `CloudSimHost/inc/visual/` |
| `KinematicsBatchScope` | FK 批末一次 `flushTransform` + Follow |
| `ensureVisual` | 统一上屏入口 |
| `visualAspectsForPropertyKey` | Data 层 schema 驱动 |

## FK / IK

- `RobotSceneKinematics`：仅 `mesh->setWorldMatrix`，OSG 读回已删除
- `RobotServiceAdapter::applyJointAnglesRad`：`KinematicsBatchScope` 包裹
- `notifyRobotKinematicsAppliedToScene`：批内仅 mark follow 脏，批末一次 solve

## 回归清单

- FK 后 Data `worldMatrix` 与 OSG outer world 一致（ε）
- 属性 pose 与 FK 交替无分叉
- Follow follower 读 target Data world
- gizmo 拖动中跳过 active backend 的 `flushTransform`
