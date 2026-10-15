# FINAL — 工程图模块清理

## 总结

已按「A + 忽略」从 CloudSim 全量移除工程图模块：独立插件、GeometryAlgorithm HLR/Drawing、Host 出图 ABI 与 HeadlessDrawingBridge、帮助页与功能专题均已清理；HOST ABI 抬至 1.57.0（`0x00013900`）。旧工程 `engineeringDrawing` 侧车字段静默忽略。

## 主要变更面

- 删除 `EngineeringDrawingPlugin` 及 `CloudSim.sln` 条目
- 删除 `HlrProject` / `DrawingGeometry` / `DrawingEngines` / `DrawingGeomFactory`
- 拆除 `projectBrepHlrToDrawing` / `projectBrepToEngineeringDrawing` 及类型
- 删除 `HeadlessDrawingBridge`；WebGateway sidecar 同步不再触及工程图
- 文档与 `help/{zh,en}` 去掉工程图工作区

## 编译

Debug|x64 与 Release|x64：GeometryAlgorithm、CloudSimCore、CloudSimPluginSDK、CloudSimHostCore、CloudSimHost、CloudSimHostHeadless、CloudSimWebGateway、Widget、GeometricModelingPlugin 均已通过。

## 任务文档

见同目录 ALIGNMENT / CONSENSUS / DESIGN / TASK / ACCEPTANCE / TODO。
