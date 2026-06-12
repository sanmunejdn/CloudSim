# 统一世界坐标契约

> **必读**：凡新增/修改导入、显示、FK、配准、拾取、坐标系叠加、TCP 示教代码，须先对照本文。权威 API 见 [`BackendSpatial.h`](../src/Data/Data/inc/BackendSpatial.h)。

---

## 1. 核心公式

```text
世界点 = objectWorldMatrix × v_stored
p_world = p_model × R(rotation) + pose          // OSG 行向量
p_world = R(rotation) × p_model + pose          // BackendMat4 列向量（经 Adapters 与 OSG 等价）
```

| 项 | 约定 |
|----|------|
| `geometry`（soup / shape / 点云） | 存**世界绝对坐标**；导入烘焙后**不再**为显示而平移顶点 |
| `pose` + `rotation` | **`pose` = 模型坐标原点在世界中的位置 (mm)**；`rotation` = 绕模型原点旋转（内禀 ZYX 度）。权威 API：`engine::rigidTransformFromBackendPoseEuler` / `backendPoseEulerFromRigidTransform`（`BackendWorldPose.h`）；包装：`backend_pose_osg::*`、`backend_world_mat_from_pose` |
| 普通 mesh/点云导入 | `pose = 0`，`rotation = 0`，顶点保持文件坐标 |
| `m_backendModelCenters` | **仅**相机聚焦与外包络；**不参与** pose 分解、拾取反算 |

### 1.1 pose / rotation 基础语义（开发必读）

本节为属性面板、`ObjectGizmoFrame`、ICP、Follow 共用的**唯一口径**；实现以 `engine::RigidTransform` 为真值，禁止各模块自写欧拉或矩阵拼装。

#### pose 是什么

| 概念 | 约定 |
|------|------|
| `pose` | **模型坐标原点** `(0,0,0)_model` 在世界系中的位置 (mm) |
| `rotation` | 将**模型系**下的点/方向变到**世界系**的旋转 R（见下） |
| 枢轴 | 旋转绕**模型原点**，不是 AABB 中心；`inner PAT` 恒 `(0,0,0)` |
| `modelCenter` | 仅相机聚焦、外包络、拾取缓存；**不参与** pose 读写 |

绕模型原点旋转时：**`rotation` 变、`pose` 不变**（R×T 语义，见 §3）。

#### 点变换：主动旋转，模型系 → 世界系

采用**主动旋转**（旋点、动刚体），不是被动「只转坐标轴、点不动」：

```text
p_world = R(rotation) × p_model + pose     // 列向量 / Eigen / BackendMat4 / ICP
p_world = p_model × R(rotation) + pose     // OSG 行向量
```

OSG 外层矩阵由 `engine::osgMatrixFromRigidTransform` 生成，等价于行向量 **`Rotate(q) × Translate(pose)`**（先转后移；与上式同一刚体）。

#### 欧拉角：内旋（intrinsic）ZYX

| 项 | 约定 |
|----|------|
| 名称 | **内禀 Tait-Bryan ZYX**（内旋） |
| 顺序 | 先绕物体 **Z** 转 `rotation.z`，再绕新 **Y** 转 `rotation.y`，再绕新 **X** 转 `rotation.x` |
| 矩阵（列） | `R = Rz(ez) · Ry(ey) · Rx(ex)` |
| 四元数 | `q = qz · qy · qx`（`BackendVisualMath` / `Adapters`） |
| 与外旋关系 | 与**外旋 XYZ**（固定世界轴 X→Y→Z）得到**同一 R**；工程文档统一称为**内旋 ZYX** |

属性面板手输 `rotation.x/y/z` 表示上述**总姿态**的 ZYX 分解，不是增量欧拉。

#### 行向量 vs 列向量

同一刚体，两层写法（**禁止**直接拷贝矩阵元素互转）：

| 层 | 乘法 | 平移在矩阵中的位置 | 模块 |
|----|------|-------------------|------|
| **列向量** | `p' = M × p` | `BackendMat4` 列主序 `v[12..14]`；`RigidTransform` 的 `translation()` | `Data`、`GeometryEngine`、ICP、`backend_world_mat_from_pose` |
| **行向量** | `p' = p × M` | OSG `m(3,0..2)`（第 3 行） | `OsgWidget`、`BackendVisual` outer、URDF FK 输出 |

桥接（`Adapters.cpp`）：`R_eigen = R_osg^T`，平移取 OSG 底行；**唯一**互转入口 `rigidTransformFromOsg` / `osgMatrixFromRigidTransform` / `colMajorFromRigidTransform`。

#### 权威 API（单一路径）

```text
engine::rigidTransformFromBackendPoseEuler(pose, rotation)
    → osgMatrixFromRigidTransform        // BackendPoseOsg / ObjectGizmoFrame / buildOuterBranch
    → colMajorFromRigidTransform         // backend_world_mat_from_pose
engine::backendPoseEulerFromRigidTransform  // 从 RigidTransform 反解 pose + euler
```

#### Gizmo 与存储语义

| 场景 | 说明 |
|------|------|
| **存盘 `pose` / `rotation`** | 物体在世界中的总位姿；`rotation` 为内旋 ZYX 分解 |
| **Gizmo World** | 交互时沿**世界轴**平移/旋转（外旋式操作） |
| **Gizmo Local** | 交互时沿**物体轴**平移/旋转（内旋式操作） |
| **写回 backend** | 总姿态 → `writeActiveBackendPoseFromOsg` → 再分解为 ZYX；与拖动方式无关 |

#### `poseReferenceFrame`（可选）

- 默认 `World`：`pose` / `rotation` 即世界系总位姿。
- `Parent`：属性面板可显示相对父对象位姿；写入 backend 前仍换算为世界系（`setPoseInFrame` / `BackendDataManager`）。

---

## 2. 矩阵约定（易错）

工程内并存三套矩阵语义，**禁止混用**：

| 语义 | 乘法 | 平移位置 | 典型用途 |
|------|------|----------|----------|
| **列主序 `BackendMat4` / `MeshBackendData::transformVertices*`** | 左乘 `M * v` | `v[12], v[13], v[14]` | 顶点烘焙、`linkMeshFileToLinkColumnMajor16` |
| **OSG `osg::Matrixd`（行向量）** | 右乘 `v * M` | `m(3,0), m(3,1), m(3,2)` | FK 输出、`setBackendRootWorldMatrixFromWorld` |
| **`mat4ToOsg` 桥接** | `C(M)=M^T`（列向量 Mat4 → 行向量 OSG） | OSG 第 3 行 | `computeMeshWorldMatrices` 导出 |

**URDF 顶点烘焙**（`UrdfRobotImport`）：

1. 用 `computeMeshWorldMatrices(..., meshVerticesAlreadyInLinkFrame=false)` 得 `Tbind`（OSG 行向量）。
2. 转列主序烘焙时须 **转置**：`out[col*4+row] = osg(col, row)`；**禁止** `m(row,col)` 直接拷贝（平移会落在错误分量，连杆散开）。
3. **禁止**先 `linkMeshFileToLinkColumnMajor16` 再烘焙完整 `Tbind`（visual origin 会被应用两次，`base_link` 等会相对臂体错位）。

---

## 3. 显示（OSG / BackendVisual）

```text
outer (MatrixTransform)     ← engine::osgMatrixFromRigidTransform(pose, rotation)；inner 枢轴 = (0,0,0)
└─ inner (PAT)              ← position = (0,0,0)（不再 -bboxCenter）
   └─ Geode（几何已世界坐标）
```

- 已删除 `skipInnerModelCenterRebase`、`m_backendSkipCenterRebase` 主路径。
- Gizmo / 拾取：直接读写 `pose` / `rotation`；**绕模型原点旋转时 `pose` 不变**。

---

## 4. 配准与派生几何

- 扫描/模板点云：先乘各自 `worldMatrix()` 到世界系做 ICP；结果**只写** `template.pose`，geometry 不变。
- `RegistrationWorldFrameSnapshot`：仅 `scanRootWorldMat` / `templateRootWorldMat`。
- 曲面重构 / B-rep 更新：输出世界几何，`pose = identity`。
- 派生件显示对齐：`inheritBrepVisualPoseFromSourceMesh` 复制源 `pose` / `rotation`。

### 4.1 CAD 模板 B-rep 配准写回

与 mesh/点云「顶点已世界烘焙」不同，CAD 模板 `shapeRef` 保持 **STEP 文件坐标**（模型系）：

| 对象 | 坐标约定 | 配准后写回 |
|------|----------|------------|
| 点云 `stored` | 世界绝对坐标 | **不变**；配准前经 snapshot 转到 STEP 模型系参与 ICP |
| 模板 backend | 模型系几何 + `pose` | **originalPose**：原始 STEP + `worldAfter`（ICP 只写 pose） |
| 面重构 cache | `alignedTemplateShape`（aligned 系） | 仅 cache；不依赖模板 backend 是否烘焙 |
| 精配 `FineOnly` | 原始 STEP soup + 当前快照扫描 | `templateToScan` 为相对当前 pose 的增量；显示仍 originalPose |

详见 [`template_brep_pointcloud_update.md`](template_brep_pointcloud_update.md) §2.1、§3.2.1。

---

## 5. URDF 每连杆（`meshVerticesInLinkFrame = false`）

### 5.1 导入（q0）

```text
1. load mesh 文件
2. 单次 Tbind 烘焙顶点到世界（见 §2）
3. pose = identity，outerBind = identity
4. setRobotPerLinkKinematicsBinding(..., meshVerticesInLinkFrame=false)
5. applyJointAnglesFromDocument(q0)
```

### 5.2 运行时 FK

```text
M_link = M0 · inv(T0) · Tq · P
```

| 符号 | 含义 |
|------|------|
| **T0** | bind 时网格世界矩阵（`fkMeshWorldT0`，来自 URDF FK） |
| **Tq** | 当前关节角下网格世界矩阵 |
| **M0** | bind 时 outer 参考（世界烘焙导入为 **I**） |
| **P** | `robotBasePlacementWorld`（整机关节链场景根位姿；gizmo 移整机**只改 P**） |

顶点已含 bind 世界位姿时，**M0=I、q=q0 ⇒ outer=I**；关节运动仅靠 `inv(T0)·Tq` 更新 outer。

**禁止**：把拖动后的世界矩阵 **W** 直接写入 **M0**（会双乘 **P**，连杆再次散开）。恢复：`M0 = W · inv(P) · inv(Tq) · T0`（`reconcilePerLinkOuterBindFromScene`）。

### 5.3 与「连杆系顶点」模式对比（`meshVerticesInLinkFrame = true`）

| 项 | 世界烘焙（当前默认） | 连杆系顶点 |
|----|----------------------|------------|
| 顶点 | 世界绝对坐标 | 连杆系 + visual 已烘焙 |
| outer @ q0 | I | 含 FK |
| 工具轴叠加 | 挂 **base_link**，`local = toolTcpInBaseFromFk(该工具)` | 挂 **法兰 link**，`local = T_flange_tool` |
| TCP 拖动罗盘 | 挂 base_link，`updateTcpDragTeachFromTarget` 跟拖动目标 | 挂法兰，`toolLocalOnFlange` 随 FK |

---

## 6. 工具 / 用户坐标系叠加

实现：`RobotSimulationController::refreshRobotCoordinateFrameOverlays` → `OsgWidget::setRobotFrameOverlays`。

| 模式 | 工具系 | 用户系 |
|------|--------|--------|
| 世界烘焙 per-link | `mount = urdfRootLinkBackendId`；`local = toolTcpInBaseFromFk(urdf, q, **该 tool**)` | `mount = base_link`；`local = T_base_user` |
| 连杆系 per-link | `mount = flangeLink`；`local = T_flange_tool` | 同左 |
| 层级 OSG | `mount` 空；`local = FK TCP` | 同左 |

**多工具**：`toolTcpInBaseFromFk` **必须**使用参数 `tool.T_flange_tool`，**禁止**经 `toolMat4ForFrames(frames, nullptr)` 取激活工具（否则多工具显示重合）。

---

## 7. TCP 末端拖动示教

- 指令/IK 仍用 **基座系** `T_base_target`；世界显示：`T_world = P × T_base_target`。
- 世界烘焙模式下拖动：
  - 每次 `applyTcpTeachTranslation*` / `Rotation*` 后调用 `updateTcpDragTeachFromTarget(target, false)` 刷新罗盘 mount。
  - IK 应用关节后：`refreshRobotCoordinateFrameOverlays()`（`m_tcpDragApplyingIk` 会屏蔽轴滑块回调中的重复刷新，须在 IK 路径末尾显式刷新）。
- `robotBaseWorldMatrixForInstance`（per-link）**必须**返回 **P**，勿用根连杆 mesh 世界矩阵当基座。

---

## 8. API 索引

| API | 模块 |
|-----|------|
| `rigidTransformFromBackendPoseEuler` / `backendPoseEulerFromRigidTransform` | `GeometryEngine/BackendWorldPose` |
| `worldMatrixFromBackendPoseEuler` / `backendPoseEulerFromWorldMatrix` | `BackendPoseOsg` |
| `objectWorldMatrix` / `transformPointToWorld` | `BackendSpatial` |
| `backend_world_mat_from_pose` | `BackendFollowMath` |
| `computeMeshWorldMatrices` / `linkMeshFileToLinkColumnMajor16` | `RobotUrdf` |
| `importUrdfRobot` | `CloudSimHost` |
| `applyJointAnglesViaLinkBackends` | `RobotScene` |
| `perLinkUsesWorldBakedMeshVertices` / `toolTcpInBaseFromFk` | `RobotWidget` |
| `setRobotFrameOverlays` / `updateTcpDragTeachFromTarget` | `OsgWidget` |

---

## 9. 反模式清单

| 禁止 | 后果 |
|------|------|
| inner PAT `-modelCenter` 参与位姿 | pose 与几何双重偏移 |
| URDF 双重烘焙 visual（fileToLink + Tbind） | 底座与臂体分离 |
| OSG→列主序不转置就烘焙顶点 | 连杆散开 |
| 世界烘焙后工具轴仍挂法兰 + 仅 `T_flange_tool` | 坐标系落在基座原点 |
| 多工具 overlay 共用激活工具矩阵 | 多工具重合 |
| 把场景 **W** 写入 **M0** | FK 双乘 **P** |
| 拾取/配准对顶点加减 `modelCenter` | 与世界契约不一致 |
| 手写 `Translate(pose)×Rotate` 或 OSG/BackendMat4 直接拷贝 | 与 `RigidTransform` 不一致；须走 §1.1 权威 API |
| 把 `pose` 当成 T×R 分解参数或模型中心世界坐标 | 与 §1.1 语义冲突；手输/旋转后视觉错位 |

---

## 10. 不兼容项（已移除）

旧工程语义不再支持：`skipInnerModelCenterRebase` 主路径、`meshInLinkFrame` 导入分支、质心 rebase 配准主路径、顶点与 pose 混合存储。
