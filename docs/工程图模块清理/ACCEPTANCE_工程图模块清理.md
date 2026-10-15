# ACCEPTANCE — 工程图模块清理

| 验收项 | 状态 | 说明 |
|--------|------|------|
| EngineeringDrawingPlugin 目录删除 | 完成 | `src/Plugins/EngineeringDrawingPlugin` 已不存在 |
| CloudSim.sln 无工程图工程 | 完成 | 无 GUID / 工程名 |
| GeometryAlgorithm 无 HLR/Drawing* | 完成 | 源文件与 vcxproj 项已删 |
| SDK/Host 出图 ABI 拆除 | 完成 | `IPluginGeometryHost` 无 drawing API；HOST `0x00013900` |
| HeadlessDrawingBridge 拆除 | 完成 | DocumentHost / WebGateway / vcxproj 已清 |
| 文档/帮助不再列可用工程图 | 完成 | features/工程图 删除；help drawing.html 删除并重生 |
| 侧车忽略 | 完成 | 无读写 `engineeringDrawing` 代码 |
| Debug\|x64 依赖链编译 | 完成 | GeometryAlgorithm → Widget + GeometricModelingPlugin |
| Release\|x64 依赖链编译 | 完成 | 同上 |

## grep 残留

活跃源码/工程/帮助中无 `com.cloudsim.drawing` / `projectBrepToEngineeringDrawing` / `HeadlessDrawingBridge` / `HlrProject` / `EngineeringDrawingPlugin`。仅本任务文档目录保留历史描述。
