# ACCEPTANCE — 场景层级块式显示

## 任务完成情况

| 任务 | 状态 | 说明 |
|------|------|------|
| T1 SceneHierarchyCanvasWidget | 完成 | 只读块+父子贝塞尔连线、自动分层布局、平移缩放 |
| T2 MainWindow 接入 | 完成 | 替换 `QTreeWidget`；`refreshOsgSceneTree` 改 `setSceneGraph` |
| T3 文档 | 完成 | `Widget/DEVELOPER_GUIDE.md` §5.3 |
| T4 编译 | 完成 | Debug\|x64 + Release\|x64 均通过 |

## 验收对照 CONSENSUS

1. 无双列表格 — ✅
2. 视觉对齐组装画布块 — ✅（160×72、圆角、左右端口、`#EEF1F5`）
3. 自动左→右布局 / 空场景提示 — ✅
4. 滚轮/中键 Alt 平移；未显示 Tab 不取快照 — ✅
5. 文档 + 双配置编译 — ✅

## 手工回归建议

1. 打开工程 → 右侧 Workspace →「场景层级」
2. 确认块与连线；选中块底栏显示本地变换摘要
3. 滚轮缩放、Alt+拖动画布、拖动块浏览
4. 切到 Units 再切回，树/画布仍刷新
