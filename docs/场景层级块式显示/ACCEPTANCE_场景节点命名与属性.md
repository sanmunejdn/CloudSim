# ACCEPTANCE — 场景节点命名与双击属性

| 项 | 状态 |
|----|------|
| SceneNodeInfo 扩展 backend/矩阵/渲染字段 | ✅ |
| 快照填 BackendIdUserData + Data displayName | ✅ |
| 画布标题用 displayName + OsgSceneNodeI18n | ✅ |
| 双击详情对话框 | ✅ |
| 有 backendId 联动 selectBackendById | ✅ |
| Debug\|x64 + Release\|x64 Widget | ✅ |

## 手工回归

1. 导入模型 →「场景层级」后端根块标题应与 Units 树对象名一致，带「对象」徽章  
2. 双击块 → 对话框含本地/世界矩阵、渲染摘要、三角面约计  
3. 双击后端根 → 3D/Units 同步选中  
4. 切换中/英 → 系统节点名与类型映射更新  
