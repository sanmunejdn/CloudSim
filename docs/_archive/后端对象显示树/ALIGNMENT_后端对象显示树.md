# ALIGNMENT：后端对象显示树

## 1. 原始需求

整理后端对象的显示框架逻辑，目标效果：

- **一个文档对应一个根节点**
- **一个后端对象对应一个子节点**（严格 1:1，无 `(ref)` 克隆）
- 按树框架组装；多打开文档同时显示在 Units 树中

交付阶段：**D2** — 先定方案文档并对齐 DEVELOPER_GUIDE，审批后再改代码。

## 2. 项目上下文

| 层 | 现状 |
|----|------|
| Data | 每文档独立 `BackendDataManager`（DAG 真源）；`BackendHierarchyChange` / `BackendHierarchyModel` 已支持增量镜像 |
| Core | `IDocumentScope` + `IDataService::listObjectSnapshots` / `BackendObjectDto`；注册/移除事件带 `documentId` |
| Host | `DocumentHost` 持有 manager + `DataServiceAdapter` |
| Widget Units | `MainWindowBackendTree::refreshBackendTree`：固定合成根 `BackendDataManager`，仅 `currentPage()`，多父造 `(ref)`；全量 `takeChildren` |

文档身份已在 Tab（`DocumentPage` / `doc-UUID`）体现，**未**进入 Units 树为根节点。

## 3. 已确认决策

| 项 | 决定 |
|----|------|
| 树形 | **C**：每个打开文档各一根；其下挂该文档对象 |
| 对象层级 | **B**：无父 → 文档根；有父 → `parentIds.front()`；禁止 `(ref)` |
| Annotations | 挂在对应**文档根**下的分组节点（与对象并列） |
| 选中非活动文档对象 | 先切 Tab / 激活 scope，再应用选中/属性/可见性 |
| 刷新策略 | **DisplayForest + 文档作用域补丁**；禁止跨文档全局全量重建 |
| 实现分期 | P0 语义+文档作用域 rebuild；P1 增量；P2 可选 Model/View |

## 4. 边界

**在范围内**

- Units 树语义与组装框架（显示投影，不改 Data DAG 真源）
- Selection / 可见性勾选 / 右键菜单 / Annotation 增量 / focus / Tab 生命周期的 document-scoped 对齐
- Core / Data / Widget / `backend_visibility` 文档对齐

**不在范围内**

- 修改 `BackendDataManager` 多父边存储或工程 JSON
- 将 OSG 场景调试树合并进 Units
- 首版强制替换为 `QAbstractItemModel`（P2 可选）
- 强制变更插件 `focusBackendInTree` API 签名（双参扩展可选）

## 5. 需求理解

显示树是 Data DAG 的**投影视图**：

- 文档根 ↔ 打开的 `DocumentPage`
- 对象节点 ↔ `BackendObjectDto`，主父边投影
- 多父仅保留在 Data；UI 不克隆次父路径
- OSG 调试树仍只反映**活动文档**（与 Units 多文档常驻语义分离）

效率上：朴素「多文档 + 每次全局 takeChildren」比现状更慢；必须文档作用域 + 结构/属性分流。

## 6. 跨功能影响矩阵

| 功能面 | 关键代码 | 风险 | 同步优化 |
|--------|----------|------|----------|
| 树组装 / i18n | `MainWindowBackendTree`、语言切换根文案 | 合成根文案失效 | 文档根 = Tab 标题；Annotations 分组 i18n |
| Tab 切换 | `onDocumentTabChanged` | 全局全量或误清其它文档 | 其它文档子树常驻；切 Tab 零对象重建 |
| 选中 / 属性 | `MainWindowSelectionService` | `currentPage` 与 item 文档不一致 | `activateDocumentForItem` 后读写 |
| 索引 | `m_backendTreeItemsById` | 单键缺 document 维度 | `(documentId, backendId)` + 文档根/注释分组映射 |
| OSG 拾取 | `selectBackendById` | 高亮缺 document | 默认当前文档；回退全局按 id |
| 可见性勾选 | `itemChanged` → `setVisible` | 写错页 Data/OSG | item 带 `documentId`；级联用该页 hierarchy |
| 右键菜单 | `onBackendTreeContextMenu` | 单一 `m_annotationRootItem` 指针比较 | 按 documentId 解析 page/rv；分组用 itemType |
| Annotations 增量 | `onAnnotationCreated/Removed/...` | 全局 annotation 根 | 按 page 挂到对应文档分组 |
| focus / 插件 | `focusBackendInTree*` | 仅 `backendId` | 兼容单参；作用域 = 活动文档 |
| 批量 suppress | `ScopedBackendTreeRefreshSuppress` | 全局全量成本上升 | 结束后对该文档一次 rebuild |
| 工程 I/O | `MainWindowProjectIo` | 低 | 加载后 rebuild 该文档；Tab 改名同步标题 |
| Robot Host | `refreshBackendTree` 转发 | URDF 落错根 | 对象落在正确文档根 |
| OSG 调试树 | `refreshOsgSceneTree` | 与 Units 多文档不一致 | **保持**仅活动文档 |
| 多父 Follow | Data 多父边 | UI 不见次父 | 真源保留；显示只投影主父 |
| 专题文档 | `backend_visibility` 等 | 描述过时 | 同步指南 |

## 7. 效率问题陈述

| 维度 | 现状 | 朴素多文档全局全量 | 目标 |
|------|------|-------------------|------|
| Register 成本 | O(当前页) | O(全部打开文档对象) | O(变更文档) |
| 切 Tab | 整树重建 | 若仍重建则浪费 | O(1) 样式 |
| visible | 易整树刷新 | 同左 | O(1) patch |
| 增量能力 | HierarchyChange 未进 Units | 仍忽略 | P1 消费事件 |

## 8. 疑问澄清（已关闭）

| 问题 | 结论 |
|------|------|
| 扁平 vs 装配层级 | B：保留主父装配层级 |
| 单文档 vs 多文档同时显示 | C：多文档各一根 |
| 交付 | D2：先文档后代码 |
| 全局全量是否可接受 | 否；锁定文档作用域框架 |
