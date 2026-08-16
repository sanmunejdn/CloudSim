# ALIGNMENT — 网页端自定义设备组装对齐桌面

## 桌面逻辑摘要

入口：左栏设备页「新建/编辑自定义设备」→ `CustomDeviceAssemblyDialog`。

### UI

| 区域 | 内容 |
|------|------|
| 顶栏 | 名称；从场景选择；导入模型；连接模式；移除；设为固定；导出 URDF |
| 画布 | Link 块 + Joint 边；连接模式父→子拖拽；唯一固定底 |
| 右侧 | 选中关节：移动/旋转、限位/home、轴 XYZ、旋转中心 Frame |

### 数据与提交

1. 首次挂几何：`ensureDevice` → `registerCustomDevice` → `attachChildToCustomDevice`
2. Link 存 `geometryBackendId` / `fixed` / `canvasX|Y`；Joint 存 `motion`（含 `motionCenterFrameBackendId`）
3. 应用：`CustomDeviceAssemblyCommit::commitGraph` 用场景世界矩阵烘焙 `restInDeviceW0` / `parentToChildRest` / `originMm`

### 约束

- 每个子 Link 至多一条入边
- 恰好一个 `fixed`（设为固定会清其它）
- Apply 需 ≥1 Link 且 ≥1 Joint，且几何可解析世界矩阵

## 网页目标

复刻上述 must-have（不含导出 URDF 品质对话框）；导出 URDF 可作为同次交付的次要项。
