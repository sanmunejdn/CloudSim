# CONSENSUS backend_dag_upgrade

## 需求共识

- 后端对象使用现有 `BackendDataBase` 统一承载。
- 后端对象关系采用 DAG（允许多父），禁止形成环。
- 后端对象支持类型化组件容器；属性组件可与通用组件并存。
- 所有对象与关系均由 `BackendDataManager` 统一管理。

## 接口共识

### 对象组件接口（BackendDataBase）

- `addComponent(std::shared_ptr<IBackendComponent>) -> bool`
- `removeComponent(const std::string& componentType) -> bool`
- `getComponent(const std::string& componentType) const -> std::shared_ptr<IBackendComponent>`
- `listComponents() const -> std::vector<std::shared_ptr<IBackendComponent>>`
- `hasComponent(const std::string& componentType) const -> bool`

### DAG 管理接口（BackendDataManager）

- `attachChild(const std::string& parentId, const std::string& childId) -> bool`
- `detachChild(const std::string& parentId, const std::string& childId) -> bool`
- `detachAllParents(const std::string& childId) -> bool`
- `parentsOf(const std::string& id) const -> std::vector<std::string>`
- `childrenOf(const std::string& id) const -> std::vector<std::string>`
- `ancestorsOf(const std::string& id) const -> std::vector<std::string>`
- `descendantsOf(const std::string& id) const -> std::vector<std::string>`
- `rootIds() const -> std::vector<std::string>`
- `wouldCreateCycle(const std::string& parentId, const std::string& childId) const -> bool`
- `validateGraph(std::string* errMsg) const -> bool`

### 查询扩展

- `findByName(const std::string& name) const`
- `findByClass(const std::string& className) const`
- `findByComponent(const std::string& componentType) const`

## 删除与关系策略

- 关系编辑默认 `detach_only`。
- 业务删除对象时使用 `collectDescendantIds + unregisterData` 显式执行。
- `unregisterData(id)` 会自动清理与该 id 关联的全部入边/出边。

## 序列化共识

- 新格式保存 `edges: [{parentId, childId}]`。
- 读取时兼容旧 `parentId` 字段：
  - 若有 `edges` 优先使用 `edges`。
  - 若无 `edges`，退回旧 `parentId` 单父逻辑。

## UI/OSG 共识

- 后端树支持 DAG：同一节点可在多个父下显示“引用节点”。
- OSG 暂采用“主父”适配（每节点选首个父关系用于现有单父渲染链）。
- 选择/显隐传播以 `descendantsOf` 为准。

## 验收标准

- 所有核心接口编译通过并能在主流程调用。
- 导入、删除、保存、打开工程流程可正常运行。
- 关系查询与环检测行为稳定且可重复验证。
