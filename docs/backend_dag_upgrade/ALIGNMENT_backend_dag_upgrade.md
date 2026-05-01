# ALIGNMENT backend_dag_upgrade

## 原始需求

- 升级后端架构，引入组件容器。
- 支持一个后端对象下挂一个或多个后端对象。
- 后端对象可获取其下挂组件与下挂后端对象。
- 所有后端对象由后端管理器统一管理，并支持查找接口。
- 后端对象组织为层级结构。

## 项目现状理解

- 当前对象主存储在 `BackendDataManager::m_records`（按 id 扁平注册）。
- 层级关系当前主要存于 `DocumentPage::m_backendParentId`（单父映射），不在 Data 层内聚。
- `MainWindowObjectRepository` 与 `MainWindowObjectGraph` 使用 `listData + backendParentId` 合成对象树。
- OSG 层使用 `m_backendParentIds` 同步层级，仅支持单父关系语义。
- 属性系统已有 `BackendAttributeBase`，但缺少通用类型化组件容器。

## 术语与规范冻结

- `BackendObject`: 继续使用现有 `BackendDataBase` 作为后端对象基类。
- `BackendComponent`: 新增类型化组件基类 `IBackendComponent`。
- `ComponentKey`: 使用字符串键（`std::string`）作为组件类型 key，避免 RTTI 跨模块风险。
- `BackendDAGEdge`: 有向边 `(parentId, childId)`。
- `BackendGraph`: 每个 `DocumentPage` 拥有一个独立 DAG，归属其 `BackendDataManager`。

## 边界确认

- 本次必须实现：
  - Data 层 DAG 关系维护接口与查询接口。
  - `BackendDataBase` 组件容器接口。
  - 仓储与对象图读取关系源迁移到 Data 层 DAG。
  - 项目序列化支持 `edges`，并兼容旧 `parentId`。
  - 文档与验收文件同步。
- 本次不做：
  - 全面重构 OSG 内部层级存储为多父结构（先以主父适配）。
  - 复杂图可视化布局算法（先保证功能正确）。

## DAG 约束与行为

- 允许多父：一个 child 可关联多个 parent。
- 禁止环：`attachChild` 前必须环检测。
- 删除默认策略：`detach_only`（仅断开边）用于关系管理；对象删除时支持 `delete_reachable_nodes`（显式调用）。
- 根节点定义：入度为 0 的节点。

## 兼容策略

- 保留 `registerData/getData/listData/unregisterData` 原语义。
- 保留 `DocumentPage::backendParentId()`，但视为兼容镜像（优先读取 Data 层关系）。
- `MainWindowObjectRepository` 维持外部接口不变，内部切换关系来源。

## 验收口径

- 关系：可多父挂接、可断边、可查父/子/祖先/后代、可环检测。
- 组件：可增删改查类型化组件。
- 兼容：旧工程文件（仅 `parentId`）可继续读取。
- UI：后端树可展示 DAG（同一对象可在多个父分支出现引用项）。
