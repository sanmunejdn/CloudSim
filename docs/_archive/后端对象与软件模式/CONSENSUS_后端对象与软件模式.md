# CONSENSUS — 后端对象与软件模式（P0）

## 需求描述

固化后端类型身份、工程侧车键与工作区模式边界，使后续模式扩展有明确「放哪里」规则，避免把非场景域模型塞进 `BackendDataBase`。

## 验收标准

1. Data DEVELOPER_GUIDE 含权威「三键」对照表；代码真源为 `CloudSimCore/BackendTypeIds.h`；新类型须三列齐全后再注册。
2. 本文与 DESIGN 列出 `project.json` 根键注册表（owner、空写策略、schema 提示）；键常量 `kProjectKey*`。
3. docs/README 写明：工作区模式只影响 UI 互斥，不卸载/不过滤 Backend 类型。
4. 明确禁止：工艺节点 / JobSet / DES 统计派生 `BackendDataBase`。
5. `catalogTypeFromClassName` 与全仓库业务比较/赋值使用 `backend_type::*`（LLM「Model」表单项等除外）。

## 技术方案（摘要）

| 层 | 约定 |
|----|------|
| 场景对象 | `BackendDataBase` 浅继承 + `BackendRegistry` + Component + Visual |
| 文档侧车 | Host 核心键 + 插件 `onProjectAboutToSave` / `onProjectLoaded` |
| 工作区 | `claimWorkspaceMode(pluginId)`；空串退出 |

扩展三路径（细则见 DESIGN）：

- **A** Host 创建内置类型（现状推荐：几何建模 → `ParametricBrepModel`）
- **B** `registerBackendType` + `PluginDelegatedBackend`（半成品；无需求不启用）
- **C** 侧车根键（工艺流程 `processFlow`、几何 UI `geometricModeling`）

## 技术约束

- 工程 `version` 仍为 `4`；侧车键演进用键内自有版本字段或文档登记，不擅自 bump 全局 version。
- 插件仅链 PluginSDK；不直链 Data/OSG。
- 新代码禁止再增 className 别名；读路径可保留既有兼容。

## 任务边界

P0 仅文档与指南。P1+（委托几何补强、跨模式 binding、CapabilityProfile）另开任务。
