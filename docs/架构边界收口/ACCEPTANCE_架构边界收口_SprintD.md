# ACCEPTANCE：架构边界收口（Sprint D）

> 日期：2026-07-21  
> 范围：`RobotOsgUiTypes` 去 osg；叠加路径仅在 `OsgWidget` 边界转 OSG

## 验收项

| ID | 项 | 证据 | 结果 |
|----|----|------|------|
| D1 | `RobotOsgUiTypes` 无 osg 头 | `OsgWidgetCore/inc/RobotOsgUiTypes.h` 仅依赖 `CoreTypes.h`（`Vec3`/`Mat4`） | **通过** |
| D2 | 渲染边界转换 | `OsgWidget.cpp` 内 `osgVec3FromCore` / `osgMat4FromCore` | **通过** |
| D3 | Host/Widget 适配直通 | `OsgRenderViewAdapter` / `WidgetOsgViewHost` 叠加字段直接拷贝 Core 类型 | **通过** |
| D4 | 调用方改写 | `RobotSimulationController` / `FeaturePickTransform` / `FeatureTrajectoryPageWidget` | **通过** |
| D5 | 编译 | `CloudSim.sln` `/t:OsgWidgetCore;CloudSimHost;RobotWidget;Widget` Debug|x64 | **通过** |

## 已知残余

- `IRobotOsgViewHost` 仍含 `osg::Vec3f`（mesh 三角高亮 / 拟合面预览）；已显式 `#include <osg/Vec3f>`（不再经 `RobotOsgUiTypes` 间接引入）
- `DocumentPage` 元数据 / `IRobotSimulationDocument` 仍含 osg；`backend()` 穿透待定
- 磁盘上仍有未编入 `CloudSimHost.vcxproj` 的 `Host/inc|source/osg` 旧副本（真源已是 `Widget/source`）

## 手工回归建议

- 指令路点轴 / 工具·用户坐标系叠加
- Raw 轨迹预览折线与稀疏 TCP 轴
- 特征目录编号叠加（FeatureTrajectory）

## 变更摘要

- `RobotOsgUiTypes`：`osg::Vec3f`/`Matrixd` → `cloudsim::core::Vec3`/`Mat4`
- `localMatrix`：由 `double[16]` 改为 `Mat4`（列主序）
- OSG 转换仅留在 `OsgWidget`（及既有 Mat4↔osg 工具函数）
