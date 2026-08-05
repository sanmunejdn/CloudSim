# TODO — feature.compose / 硬化 P2（Host 规范收口）

## 部署

宿主 + 插件 + GeometryAlgorithm + Data 同编 **1.47.0+**（`0x00012F00`）。

## Host 规范（本轮已修）

| 项 | 说明 |
|----|------|
| ABI 虚表 | `preview/circularPattern*` 尾部追加（勿插在 linear 与 mirror 之间） |
| feature.compose LLM | `AiLlmClient` 独立解析 + `FeatureComposeDomainHandler::validatePlanJson` |
| ActionPlan | `execute` 前对 feature.compose 强制校验 |
| Pattern tip | Host 统一 `resolvePatternSeed`（与 Data rebuild 一致） |
| AI Loft | 轮廓按 plane 抬到世界坐标 |

## 已知债

| 项 | 说明 |
|----|------|
| `CloudSimPluginHost.vcxproj` | 仍含 `PluginManager` 等源；真源编译以 `CloudSimHost.vcxproj` 为准（见 DEVELOPER_GUIDE） |
| Offset | bevel 后仍自交则拒绝 |
| CircularPattern AI 轴 | 仅数值 `axis_*`；边拾取走插件 UI |
