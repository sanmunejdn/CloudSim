# FINAL · 轨迹编辑模板与撤销

## 交付摘要

- 草稿流水线独立撤销栈 `PipelineDraftEditStack`；页 Undo/Redo 优先草稿
- 命名多模板库 `UserTemplateLibrary`（流水线 + CAD 离散）；导入/导出；旧 QSettings 迁移
- 草稿期不再 `syncPipelineToBoundPathPlan`；Apply Undo 后 session/UI/raw/phase 对齐
- `DEVELOPER_GUIDE.md` 已同步；6A 文档见本目录

## 主要文件

- `UserTemplateLibrary.*` / `PipelineDraftEditStack.*`
- `TrajectoryEditPageWidget.*` / `TrajectoryEditSession.*`
- `FeatureTrajectoryPageWidget.*`
