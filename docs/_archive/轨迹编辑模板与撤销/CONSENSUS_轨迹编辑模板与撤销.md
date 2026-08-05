# CONSENSUS · 轨迹编辑模板与撤销

## 需求描述

1. 草稿流水线可撤销/重做（与程序 Apply 栈分离）。
2. 流水线与 CAD 离散参数均为命名多模板，支持保存/删除/导入/导出。
3. Apply Undo 后 PathPlan、session、UI 一致。

## 验收标准

- [ ] 加块 → 改参 → 撤销回到加块前 → 再撤销去掉块；程序树不变
- [ ] Apply → 撤销：程序内容、PathPlan.pipeline/raw/phase、session、UI 一致；可 Redo
- [ ] 流水线保存两套命名模板并可切换加载；文件导入/导出可用
- [ ] CAD 保存两套策略参数模板；加载后写回并触发重离散
- [ ] 旧 `pipelineJson` 迁移为「迁移的上次保存」
- [ ] RobotWidget Debug|x64 与 Release|x64 编译通过

## 技术约束

- 图标走 `UiIconDecorators`；不硬编码 `:/cloudsim/icons/`
- 输出目录以 vcxproj 为准，不擅自改 OutDir
- 注释纯中文、聚焦 Why
