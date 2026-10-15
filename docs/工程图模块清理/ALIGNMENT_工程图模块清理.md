# ALIGNMENT — 工程图模块清理

## 原始需求

按 CloudSim README 中的工程图能力，全量清理工程图模块的所有逻辑与代码。

## 项目上下文

- 桌面互斥工作区模式之一：`com.cloudsim.drawing`（EngineeringDrawingPlugin）
- HLR 出图在 GeometryAlgorithm（HlrProject / Drawing*）
- Host：`projectBrepToEngineeringDrawing` + `HeadlessDrawingBridge`
- 网页端此前已裁剪 UI/REST，仅残留 sidecar 桥

## 边界确认

- **做**：删除插件、HLR、Host ABI、HeadlessDrawingBridge、帮助与专题文档引用
- **不做**：侧车字段剥离迁移；归档到 `_archive`；改安装包 dist 历史产物
- **不碰**：几何建模、工艺流程、拔模（draftFaces）

## 已确认决策

- 深度：**A 全量删除**
- 侧车：`engineeringDrawing` **静默忽略**（不读写、不剥离）
