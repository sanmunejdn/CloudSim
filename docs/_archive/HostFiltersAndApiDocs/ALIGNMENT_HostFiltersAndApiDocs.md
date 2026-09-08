# ALIGNMENT — Host 筛选器整理与 API 文档

## 原始需求

1. 整理 `CloudSim/src/Host/CloudSimHost/CloudSimHost.vcxproj.filters`
2. 更新 `DEVELOPER_GUIDE.md`，详细说明 Host 目前支持的全部 API

## 项目理解

- `CloudSimHost` 为本地引擎宿主 DLL；对外稳定面为 Bootstrap / `CloudSimHost.h` / `DocumentHost` + Core 三件套；大量编排自由函数与 Headless/IO 共路。
- `vcxproj` 已包含 `source/io/*`、`DesignParts*` 等，但 filters 缺项，IO QtMoc 误挂在 `inc` 根下。
- 既有 [`INTERFACE_CATALOG.md`](../HostOptimization/INTERFACE_CATALOG.md) 为稳定面索引；`DEVELOPER_GUIDE` 叙事完整但缺系统化「全量 API」专节，且部分表述过时（指令属性「计划中」、kinematics merge「已移除」）。

## 边界

| 做 | 不做 |
|----|------|
| filters 与 vcxproj 1:1，按域分组（含 `inc\io` / `src\io`） | 不改 vcxproj 编译列表、不物理搬文件 |
| DEVELOPER_GUIDE 增补 §10 全量 API + 修正过时句 | 不重写 Plugin/AiSDK 全文；不改代码逻辑 |
| 同步 INTERFACE_CATALOG 头文件表补 `io/` | 不生成独立 OpenAPI |

## 假设（已按此执行）

1. 筛选器域与磁盘逻辑域对齐：Host 自有 `import/project/robot/headless/follow/io/adapters`；外部源在 `External\UI\…`。
2. 「全部 API」= 工厂/C ABI + DocumentHost + `IDataService`/`IRobotService`/`IRenderView` 全方法 + Host 公开自由函数/Headless/IO；插件/AI 列入口并指向专属指南。
3. `UrdfRobotImport` 头挂 `inc\robot`（与 `source\robot` 一致）。
