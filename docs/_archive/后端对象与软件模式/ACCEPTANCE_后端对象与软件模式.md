# ACCEPTANCE — 后端对象与软件模式（P0 完整落地）

## 范围

P0 文档 + 类型/侧车键代码真源 + 全仓库业务字面量收口。

## 检查项

| # | 项 | 状态 |
|---|----|------|
| 1 | ALIGNMENT / CONSENSUS / DESIGN 已落盘 | 通过 |
| 2 | Data DEVELOPER_GUIDE / docs/README / docs 索引 | 通过 |
| 3 | 契约真源 `CloudSimCore/inc/BackendTypeIds.h`；Data `BackendTypeIdentity.h` 转发 | 通过 |
| 4 | builtins / className() / Visual / ProjectIo 接入 | 通过 |
| 5 | Host / Widget / RobotWidget / PluginHost / AiWidget / 相关插件字面量改为 `backend_type::*` | 通过 |
| 6 | 侧车键 `kProjectKeyProcessFlow` / `kProjectKeyGeometricModeling` 接入 ProcessFlow / GeometricModeling | 通过 |
| 7 | PointNetDomainHandler Mesh/Brep className 误判已修 | 通过 |
| 8 | 插件工程 AdditionalIncludeDirectories 含 `CloudSimCore/inc` | 通过 |

## 故意保留

- `AiLlmSettingsDialog` 的 `QStringLiteral("Model")`：LLM 模型表单项，非后端类型
- `AiConfirmPanel` 的 AI filter 名 `Mesh` / `Brep` / `PointCloudOrMesh` 等：助手 schema 枚举，非 catalog
- 用户可见错误文案中的类型名（非比较逻辑）

## 未做（非 P0）

- 侧车键运行时 Registry API
- PluginDelegatedBackend 几何补强
- 工艺 binding 实现
- CapabilityProfile / 产品 SKU
