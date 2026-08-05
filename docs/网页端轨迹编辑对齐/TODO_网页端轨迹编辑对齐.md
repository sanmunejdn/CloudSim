# FINAL — 网页端轨迹编辑对齐

网页「轨迹编辑」已按桌面 `TrajectoryEditPageWidget` 信息架构对齐：工艺模板+生成、程序/组、中文算子双栏流水线（默认未启用）、参数、预览应用、流水线模板导入导出。

## 新增 API
- `GET /api/trajectory/op-palette`
- `POST /api/trajectory/emit`

## 待办
- 拖拽调色板入流水线、列表内拖拽重排（当前右键上移/下移）
- 投影/非刚体专用 backend 下拉的桌面级集成
- 手测完整流程
