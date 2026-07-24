# CONSENSUS：后端对象显示树

## 1. 明确需求

Units Dock 中的后端对象树按以下语义组装：

1. Units 树**只显示当前文档**的结构（一个文档根，标题与当前 Tab 一致）；切 Tab 时换显
2. 每个后端对象对应**恰好一个**树节点（无 `(ref)` 克隆）
3. 对象挂接：无父 → 文档根；有父 → 主父 `parentIds.front()`
4. Annotations 挂在文档根下的分组节点
5. 非当前文档不进 Units 树；后台文档的注册/删除事件不刷新树（切回该 Tab 时再 rebuild）

## 2. 技术方案

| 项 | 约定 |
|----|------|
| 显示投影 | PrimaryParentProjection；不改 Data DAG / 工程 JSON |
| 框架 | DisplayForest（无 Qt）+ DocumentScopedBinder（绑 QTreeWidget） |
| Item 元数据 | `kItemTypeDocument` / `kItemTypeAnnotationGroup` / `kRoleDocumentId`；对象仍用 `kItemTypeBackend` + `kRoleBackendId` |
| 索引 | `(documentId, backendId) → item`；`documentId → docRoot / annotationGroup` |
| 结构更新 | 仅 `rebuildDocument(docId)` 或 P1 局部补丁；**禁止**跨文档全局 `takeChildren` |
| 属性更新 | visible / 显示名 → `patchObjectItem`（O(1)） |
| 切 Tab | 只保留当前文档根并 rebuild；非当前文档从树移除 |
| OSG 调试树 | 仍只反映活动文档；与 Units 语义分离 |
| 插件 focus | 单参 API 兼容；作用域 = 活动文档 |

## 3. 效率原则

1. **文档作用域**：结构变更只动该 `documentId` 子树  
2. **结构 / 属性分流**  
3. **切 Tab 零结构成本**  
4. **批量合并**：`ScopedBackendTreeRefreshSuppress` 结束后对该文档一次 rebuild  

## 4. 技术约束与集成

- Widget 内实现组装器；Core **不**引入 Qt 树类型  
- 消费已有 `BackendObjectRegistered/Removed`（含 `documentId`）与 Data `BackendHierarchyChange`（P1）  
- Selection / ContextMenu / Visibility / Annotation / focus 与树改造**同批** document-scoped  
- 详见 [DESIGN](DESIGN_后端对象显示树.md)、影响矩阵见 [ALIGNMENT §6](ALIGNMENT_后端对象显示树.md)

## 5. 任务边界

**做**：P0 语义 + 文档作用域；联动改造；文档对齐。P1 增量。P2 可选 Model/View。  

**不做**：改多父真源；合并 OSG 树；强制改插件 API 签名。

## 6. 验收标准

### 树形

1. 打开 N 个文档 → Units **仅**显示当前文档一个文档根；切 Tab 后树随当前文档切换  
2. 非当前文档对象不出现在 Units 树  
3. 同一 `(documentId, backendId)` 仅一个节点（无 `(ref)`）  
4. 无父挂文档根；有父挂主父  

### 效率

5. 非当前文档的注册/删除事件不改写 Units 树（切回该 Tab 时 rebuild）  
6. 切 Tab 换显当前文档子树（`retainOnlyDocument` + sync）  
7. 可见性勾选不触发该文档整树重建（P0 目标；P1 强制）  

### 联动

8. Annotation 创建/删除/显隐只影响当前文档分组  
9. 选中/勾选/删除/导出作用在当前文档 `IDataService` / `IRenderView`  
10. `focusBackendInTree*` / 插件导入后能高亮当前文档内对象  
11. OSG 调试树与 Units 均仅当前文档  
12. Core / Data / Widget DEVELOPER_GUIDE 与本共识一致  

## 7. 不确定性

全部已关闭（见 ALIGNMENT §8）。本共识作为实现与回归的唯一验收依据；代码实现须待本包文档审批通过后启动（见 TASK）。
