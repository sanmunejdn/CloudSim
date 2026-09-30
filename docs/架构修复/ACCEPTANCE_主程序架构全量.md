# ACCEPTANCE：主程序架构全量

总单 BugTracker **#12**。

## 验收清单

| ID | 项 | 状态 | 证据 |
|----|----|------|------|
| W0 | Facade/DocumentHost/`sceneFacade` 用 SceneOps；RobotHost 门闩 `sceneOps()` | ✅ | `BackendSceneDocumentFacade` 无 `IOsgWidgetView*` 成员 |
| W1 | SelectionOperation 经 `IViewportInteractionHost`；无 `ownerWidget`/`static_cast<OsgWidget*>` | ✅ | Selection 源 grep 清零；Viewport+Host 双配置 |
| W2 | Robot 四分面 + 单分面收窄；KEEP_FAT 保留 | ✅ | `IRobotOsg{SceneOps,Pick,Overlay,Teach}.h`；DEVELOPER_GUIDE |
| W3 | 删除死 `widgetOsgFromPage` / `osgWidgetFrom` | ✅ | 仅定义移除；保留分面 helper / `backendManagerOf` |
| W4 | WHOLEARCHIVE 终态保留文档 | ✅ | Headless 设计 §9 |
| 双配置 | Host/Viewport/Widget/RobotWidget/CloudSim | ✅ | Debug 已绿；Release 见构建记录 |
| 门禁 | make-source-check | ✅ | 全步骤 OK（含 host-headless / export-surface / header-guards） |

## 冒烟（人工）

点选 / 折线 / 网格元素 / 标注 / 物体 gizmo / TCP 示教 / 切面拖拽；机器人工具条拾取与 overlay 预览。

## KEEP_FAT 列表（摘要）

`RobotSimulationController` 多面会话、`FeatureTrajectory`/`TrajectoryEdit` 预览、`AssemblyMatePanel`、`DevicePoseMotionPlayer`、`MainWindow::activeOsgViewHost()`。

## 明确未做

消灭 WHOLEARCHIVE；Robot overlay 并入 Host Viewport ABI；Playwright。
