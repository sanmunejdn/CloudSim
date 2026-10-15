# CONSENSUS — 工程图模块清理

## 需求与验收

1. 产品中不再存在工程图工作区模式与 `com.cloudsim.drawing` 插件
2. GeometryAlgorithm 无 HLR/Drawing 专用源码与工程项
3. `IPluginGeometryHost` 无出图 API；Host 无 `HeadlessDrawingBridge`
4. 文档/帮助/README 不再将工程图列为可用能力
5. Debug|x64 与 Release|x64 相关工程编译通过
6. 含旧 `engineeringDrawing` 字段的工程可打开且不崩溃；不再读写该键

## 技术方案

见同目录 DESIGN；执行任务见 TASK。HOST ABI 抬升至 1.57.0（`0x00013900`）。

## 约束

- OutDir / 插件输出路径不擅自改动
- 双配置编译验证
- 不误删 GeometricModeling 的 SketchGeom/SketchTools
