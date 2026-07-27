# ACCEPTANCE：后端对象显示树

## D0 文档交付（2026-07-23）

| 交付物 | 状态 |
|--------|------|
| ALIGNMENT / CONSENSUS / DESIGN / TASK | 完成 |
| Core / Data / Widget / backend_visibility 指南 | 完成 |

## P0 代码落地（2026-07-23）

| 项 | 状态 |
|----|------|
| `BackendUnitsDisplayForest` 主父投影 | 完成 |
| `BackendUnitsTreeBinder` 文档作用域 sync | 完成 |
| 去掉合成根 / `(ref)`；多文档 top-level | 完成 |
| Register/Remove → `rebuildUnitsDocument(docId)` | 完成 |
| 切 Tab 仅活动样式 + OSG 树（不毁其它文档节点） | 完成 |
| Selection / Visibility / ContextMenu document-scoped | 完成 |
| Annotation 按文档分组增量 | 完成 |
| Widget Debug\|x64 编译通过 | 完成（`BuildProjectReferences=false`） |

### 手动回归建议

1. 打开两个文档，Units **只显示**当前文档；切 Tab 后树内容切换  
2. 在文档 A 导入对象时，若当前是 B，Units 不变；切回 A 后可见新对象  
3. 勾选可见性写回当前文档；无 `(ref)` 节点  
4. Annotation 创建/删除落在当前文档分组  
5. 关闭文档后树显示新的当前文档  

## 门控

- [x] D2 方案文档  
- [x] P0 编码与编译  
- [ ] P1 增量（hierarchy 局部补丁）— 未做  
- [ ] 全量解决方案依赖链编译（RobotScene 等环境 lib 路径问题与本改无关）  
