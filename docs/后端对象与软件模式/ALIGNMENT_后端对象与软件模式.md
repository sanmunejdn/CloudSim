# ALIGNMENT — 后端对象与软件模式（P0）

## 项目上下文

CloudSim 以 `Data.dll` 的 `BackendDataBase` 为三维场景对象真源，经 `BackendRegistry` / `IDataService` 持久化与 CRUD；插件通过 `claimWorkspaceMode` 互斥切换工作区 UI。工艺流程与几何建模编辑态分别落在文档侧车键与 Host 几何 API，不全量进 Backend 继承树。

参考：[ARCHITECTURE_SUMMARY.md](../../ARCHITECTURE_SUMMARY.md)、[Data/DEVELOPER_GUIDE.md](../../src/Data/Data/DEVELOPER_GUIDE.md)、[ProcessFlowPlugin/README.md](../../src/Plugins/ProcessFlowPlugin/README.md)、[GeometricModelingPlugin/README.md](../../src/Plugins/GeometricModelingPlugin/README.md)。

## 原始需求

理清后端对象派生逻辑与框架；评估能否支撑不同软件工作区模式；落地 P0 契约与命名收口（文档权威化，本阶段不改运行时代码）。

## 边界确认

| 纳入 | 不纳入（本阶段） |
|------|------------------|
| className / catalog|sourceType / C++ 类型对照表 | 重命名 className、消除历史别名（代码 breaking） |
| `project.json` 根键侧车注册表 | 实现侧车运行时 Registry API |
| 工作区模式 ≠ Backend 类型过滤器 | CapabilityProfile / 产品 SKU |
| 扩展路径原则（内置 / 委托 / 侧车） | 强化 `PluginDelegatedBackend` 几何能力 |
| | 工艺节点 ↔ backendId 绑定实现 |

## 需求理解

1. 「软件模式」指互斥工作区：默认三维、工艺流程（`com.cloudsim.processflow`）、几何建模（`com.cloudsim.geomodeling`）。
2. Backend 框架只承载可进 3D 场景、可被选中/跟随/可视化的对象；流程图、DES、编辑会话 UI 状态属侧车或插件本地。
3. 命名三键长期并存：`className` 权威于工厂与工程；`sourceType`/`catalogTypeName` 用于导入与树分类；C++ 类名不必与 className 字符串相同。

## 疑问澄清（已决策）

| 问题 | 决策 |
|------|------|
| 工艺节点是否应派生 `BackendDataBase`？ | 否；继续 `processFlow` 侧车 |
| 几何建模 Body 落点？ | `ParametricBrepModel` 进 Data；插件侧车仅存 UI 态（如 `activeBodyId`） |
| P0 是否改代码？ | 否；文档 + 指南补丁为准 |
| Visual 别名 `MeshBackendData`？ | 仅兼容读路径；新写入只用 `Model` |
