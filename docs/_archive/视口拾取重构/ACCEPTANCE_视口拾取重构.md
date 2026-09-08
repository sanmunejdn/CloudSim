# ACCEPTANCE — 视口拾取重构

## 编译

- [x] Debug|x64：`Widget.vcxproj` → `bin\x64d\Widget.dll` / `CloudSimHost.dll`
- [x] Release|x64：见 `_build_release.txt`

## 架构

- [x] `IViewportPickEngine` + `OsgWidgetPickEngine` 统一 `queryPick`/高亮
- [x] `ViewportHit` + Policy（GizmoAxis / Passthrough / RobotObjectSelect）
- [x] `ViewportInteractionController`：overlays → activeTool；Session 分发
- [x] `OsgWidget::eventFilter` 拾取链改为 `controller->handleEvent`
- [x] `set*Mode` 门面同步 `setActiveTool`
- [x] meshPickCommitted：有 Session 时先 `dispatchCommit`，避免双投
- [x] 新文件入 `Widget.vcxproj` + filters；Host 同步编译 Engine/Controller

## 手工回归（建议）

- [ ] 对象选整机 gizmo；树/3D 同步；Esc
- [ ] 点云点选；折线；Mesh 边/面；Labeling
- [ ] 轨迹特征拾取；装配配合；路点；TCP/截面/罗盘
