# ACCEPTANCE：架构边界收口（Sprint E）

> 日期：2026-07-21  
> 范围：`IRobotOsgViewHost` mesh 高亮/拟合面预览去 osg；确认 `Host/osg` 死副本已不在磁盘

## 验收项

| ID | 项 | 证据 | 结果 |
|----|----|------|------|
| E1 | mesh 高亮 API 用 Core Vec3 | `IRobotOsgViewHost::showMeshTriangleHighlight` / `showMeshFittedSurfacePreview` | **通过** |
| E2 | 接口头无 osg | `IRobotOsgViewHost.h` 已移除 `#include <osg/Vec3f>` | **通过** |
| E3 | 工具输出 Core Vec3 | `MeshTriangleSelectionUtil` 世界顶点改为 `cloudsim::core::Vec3` | **通过** |
| E4 | 边界转换 | `WidgetOsgViewHost` 内 Vec3 → `osg::Vec3f` 再调 OsgScene | **通过** |
| E5 | Host/osg 死副本 | `Host/inc|source/osg` 不存在；vcxproj 仅编 `Widget/source/OsgWidget*` | **通过**（已清） |
| E6 | 编译 | `CloudSim.sln` `/t:CloudSimHost;RobotWidget;Widget` Debug\|x64 | **通过** |

## 已知残余

- `DocumentPage` / `IRobotSimulationDocument` 元数据仍含 osg（关节 MT、FK 表）
- `DocumentPage::backend()` / `BackendDataManager` 穿透待定
- `IRobotBackendPoseSink` 等仍可能含 osg

## 手工回归建议

- Mesh 轨迹页：选中三角面高亮
- NURBS/B样条区域拟合面绿色预览
- 折线/刷选拾取后高亮刷新

## 变更摘要

- `IRobotOsgViewHost` mesh 叠加顶点：`osg::Vec3f` → `cloudsim::core::Vec3`
- `MeshTriangleSelectionUtil` 输出对齐 Core
- OSG 转换仅在 `WidgetOsgViewHost` → `OsgWidget`/`OsgScene`
