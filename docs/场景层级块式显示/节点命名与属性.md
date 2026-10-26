# CONSENSUS 补充 — 场景节点命名与双击属性

| 项 | 结论 |
|----|------|
| 命名真源 | 有 `BackendIdUserData` → Data `objectSnapshot.name`；否则 OSG `getName` |
| 契约 | `SceneNodeInfo` 扩展 backend/矩阵/渲染轻量字段 |
| 双击 | 画布弹只读详情对话框；有 backendId 时联动 3D/Units 选中 |
| 中文 | UI 层 `OsgSceneNodeI18n` 映射（原 BackendTree 未接线映射落地） |
