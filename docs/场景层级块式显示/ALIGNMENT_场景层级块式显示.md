# ALIGNMENT — 场景层级块式显示

## 原始需求

> `CloudSim/src/UI/Widget/DEVELOPER_GUIDE.md` 修改场景层级页面的显示形式，现在是表格的形式，改为自定义设备同样的块形式

## 项目上下文

| 项 | 现状 |
|----|------|
| 目标页 | 右侧 Workspace Dock → Tab「场景层级 / Scene graph」 |
| 当前实现 | `MainWindow::m_osgSceneTree`（`QTreeWidget`，2 列：节点名 + 本地变换） |
| 数据源 | `IRenderView::sceneGraphSnapshot(256)`，只读 OSG 调试快照 |
| 交互 | 无选中/右键/双击接线 |
| 参考「块」A | Property Dock「设备」`DevicePageWidget`：`QScrollArea` + `QGridLayout` + 96×88 `QToolButton` 缩略图瓦片 |
| 参考「块」B | `CustomDeviceAssemblyCanvasWidget`：画布上的 Link 矩形块 + 运动副连线 |

文档约定（`Widget/DEVELOPER_GUIDE.md` §5.3）：Units = 多文档后端投影；场景层级 = 仅活动文档 OSG 快照。

## 边界确认（拟定）

**范围内**

- 桌面端「场景层级」Tab 的展示控件形态改造
- 同步更新 `Widget/DEVELOPER_GUIDE.md` 相关说明
- Debug\|x64 + Release\|x64 编译验证

**范围外（默认，待确认）**

- Units 树、仿真 Dock、网页端
- OSG 运行时层级逻辑（`OsgSceneHierarchy.cpp`）
- 为场景节点新增选择/可见性等业务交互（除非明确要求）

## 需求理解

将「场景层级」从双列表格树，改为与「自定义设备 / 设备页」一致的块式视觉；数据仍来自活动文档 `sceneGraphSnapshot`，默认保持只读调试页定位。

## 疑问澄清（已解决）

### Q1「块形式」指哪一种？
- **B**（已确认）：自定义设备组装画布的 Link 矩形块风格

### Q2 层级关系如何呈现？
- **3**（已确认）：画布 + 连线

其余见 `CONSENSUS_场景层级块式显示.md`。
