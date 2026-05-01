# FINAL backend_dag_upgrade

## 交付概览

本次完成了从“扁平对象注册 + 外部单父映射”到“Data 层内聚 DAG + 类型化组件容器”的架构升级，并保持主要调用面兼容，重点实现如下：

- Data 层
  - 新增 `IBackendComponent`。
  - `BackendDataBase` 增加组件容器（add/remove/get/list/has）。
  - `BackendDataManager` 增加 DAG 关系索引、关系 API、图校验与查询扩展。
- Widget 层
  - `MainWindowObjectRepository` 改为消费 `BackendDataManager::listEdges()`。
  - `MainWindowObjectGraph` 升级为多父模型（`parentIds`）。
  - `MainWindowBackendTree` 支持引用节点 `(ref)`。
  - `DocumentPage::removeBackendSubtree` 改为 DAG 后代删除逻辑。
- IO 层
  - `MainWindowProjectIo` 增加 `edges` 序列化。
  - 打开工程支持 `edges` 优先、`parentId` 兼容。

## 关键文件

- `Data/inc/BackendComponent.h`
- `Data/inc/BackendDataBase.h`
- `Data/source/BackendDataBase.cpp`
- `Data/inc/BackendDataManager.h`
- `Data/source/BackendDataManager.cpp`
- `Widget/inc/MainWindowObjectGraph.h`
- `Widget/source/MainWindowObjectGraph.cpp`
- `Widget/source/MainWindowObjectRepository.cpp`
- `Widget/source/MainWindowProjectIo.cpp`
- `Widget/source/MainWindowBackendTree.cpp`
- `Widget/source/MainWindowFileImport.cpp`
- `Widget/source/DocumentPage.cpp`

## 结果评估

- 架构一致性：关系模型下沉到 Data 管理层，降低了 UI 与关系存储耦合。
- 可维护性：新增统一 DAG API 后，后续可在同一处强化规则（权限、事务、批量更新）。
- 风险控制：保留 legacy `backendParentId` 镜像，减少一次性切换风险。
