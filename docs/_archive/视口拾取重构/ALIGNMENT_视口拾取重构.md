# ALIGNMENT — 视口拾取重构

## 原始需求

将视口拾取拆为 PointerTool（手势/命中）与 Op/Session（业务消费），经统一 PickEngine 与 HitPolicy，完整落地全部现有拾取模式。

## 边界

| 做 | 不做 |
|----|------|
| Widget 内 Controller / Tool / Overlay / Session / Policy | 重写 `queryPick` / BREP 索引算法 |
| 全部模式迁入新框架；旧 `set*Mode` 保留为门面 | 选择策略下沉进 OsgWidgetCore |
| 新文件入 vcxproj + filters；C++/注释规范 | 强制 ViewportHit 立刻进 Contracts |

## 调用方清单

对象选、点云点选、折线、Mesh 边/面、Labeling、指令路点、TCP/截面/对象罗盘；轨迹/配合/选件/插件 Session。
