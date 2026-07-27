# ACCEPTANCE — 几何建模 SDK 插件

## 检查清单

| 项 | 状态 | 说明 |
|----|------|------|
| 6A 文档 | 完成 | ALIGNMENT/CONSENSUS/DESIGN/TASK |
| PlaneGCS vendor | 完成 | `third_party/planegcs` + ORIGIN.md；启动等边三角形自测 |
| SketchExtrude | 完成 | `geoalgo::sketchExtrude*` + SelfTest Pad/Pocket |
| OneCAD 复制 | 完成 | `ported/onecad/{sketch,commands,history,modeling}` + 适配 CommandStack/FeatureDocument |
| Host API 1.21.0 | 完成 | embed 视口、face plane、overlay、extrude；双 null 仍显示 AI |
| 插件菜单/布局 | 完成 | 几何建模进入/返回；中央页嵌 3D；仅 AI |
| 选面草图 | 完成 | pick face → 平面草图 + 默认轮廓 + 约束求解 |
| Pad/Pocket rebuild | 完成 | 经 Host extrude → BrepModel |
| Undo | 完成 | CommandStack（OneCAD 模式，深度 200） |
| 工程钩子 | 完成 | `geometricModeling` JSON |

## 编译验证

请在 VS 生成：`GeometryAlgorithm`、`CloudSimPluginSDK`、`CloudSimHost`、`Widget`、`GeometricModelingPlugin`（Debug|x64）。

## 手工验收步骤

1. 启动 CloudSim，打开含 B-rep 的文档  
2. 菜单「几何建模 → 进入几何建模」：左右栏隐藏，仅 AI，中区 3D  
3. 「新建草图」→ 拾取平面面 → 求解 → Pad  
4. 「返回三维场景」恢复布局  
