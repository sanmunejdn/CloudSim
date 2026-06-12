# 统一世界坐标契约 v2（Breaking Change）

> **必读**：凡新增/修改导入、显示、FK、配准、拾取、面重构代码，须先对照本文。  
> **v2 不兼容旧 JSON**：不再读写独立 `pose`/`rotation` 字段，仅 `worldMatrix`（16 元列主序 `BackendMat4`）。

---

## 1. 数据模型

```text
Object {
  Geometry geometry;        // 出生时坐标，用户操作不改顶点
  BackendMat4 worldMatrix;  // 唯一权威位姿
}
p_world = p_geometry × worldMatrix   // OSG 行向量；BackendMat4 经 Adapters 与 OSG 等价
```

属性面板 `pose`/`rotation` 为 **worldMatrix 分解视图**，非独立存储。

---

## 2. 七条规则

| 规则 | 约定 |
|------|------|
| **出生** | STEP/PLY/点云/mesh：`geometry`=文件坐标，`worldMatrix`=I（URDF 连杆 FK 初值写入 M） |
| **移动** | 用户拖/Gizmo：`worldMatrix = T_inc × worldMatrix`；geometry 不变 |
| **算法** | 需要世界坐标：`v_world = v_geometry × worldMatrix`；输出新对象：`geometry`=算法结果，`worldMatrix`=I |
| **显示** | inner PAT=I；outer=localFromWorld(`worldMatrix`) |
| **属性面板** | 改 rotation 时固定分解平移，重建 M；gizmo 拖动中 UI 读 OSG gizmo（`syncPropertyPanelGizmoLiveValues`），松手 `commitGizmoPoseToBackend` 写回 `worldMatrix` |
| **STEP 导出** | 直接 `exportSTEP(geometry)`；几何未改则即文件坐标 |
| **面归属 A** | shape=文件坐标；`v_model = v_world × inv(template.worldMatrix)` |

---

## 3. 配准（CAD 模板 + 点云）

```text
scan_world    = scan.geometry × scan.worldMatrix
template_world= template.soup × template.worldMatrix
ICP → deltaM（世界系）
template.worldMatrix = deltaM × template.worldMatrix
scan / template geometry 均不变
```

禁止 ICP 烘焙进 `geometry`（删除 `alignedTemplateShape` 配准路径）。

---

## 4. 面重构

```text
v_world = scan.geometry × scan.worldMatrix
v_model = v_world × inv(template.worldMatrix)
updateShapeFromPointCloud(originalSTEP, v_model)
new.geometry = 调整后 STEP（文件系）
new.worldMatrix = template.worldMatrix
```

---

## 5. URDF 例外

连杆：`geometry`=link 文件系；`worldMatrix` 每帧 FK 刷新。

---

## 6. API

| API | 路径 |
|-----|------|
| 权威存储 | `BackendDataBase::worldMatrix()` / `setWorldMatrix()` |
| 点变换 | `BackendSpatial::transformPointToWorld` / `transformPointToStored` |
| UI 分解 | `backend_pose_euler_from_world_mat` / `backend_world_mat_from_pose` |
| OSG 桥接 | `backend_pose_osg::osgMatrixFromBackendWorldMatrix` |

---

## 7. 嵌套 OSG 父链

backend 存**绝对** `worldMatrix`；`setBackendRootWorldMatrixFromWorld` 写 OSG local = world × inv(parentWorld)。
