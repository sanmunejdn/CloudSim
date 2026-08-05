# ALIGNMENT — 网页端轨迹编辑对齐桌面

## 原始需求
同步网页「轨迹编辑」与桌面端一致（UI + 逻辑）。桌面参考：`TrajectoryEditPageWidget`。

## 范围
- 对齐：工艺模板行、程序/组、中文算子调色板、左右双栏流水线、参数区、预览/应用/重置/撤销/重做、流水线模板（含导入导出）、**生成(Emit)**、新建算子默认未启用
- 不改：轨迹生成 CAD/Mesh（已对齐）、桌面零回归

## 理解
网页已有 recipe/pipeline/preview/apply API，但布局与桌面 IA 不一致；缺 Emit；新建 op 默认 `enabled:true`（桌面为 false）。

## 关键决策（已按「与桌面一致」默认）
1. 完整对齐布局 + 逻辑（含生成/启用默认关/模板导入导出）
2. 程序/组：顶栏选择；组写入算子 scope（能绑则绑）
3. 调色板顺序/中文名以 Host `trajectoryOpPaletteKinds` + `displayName(true)` 为准
