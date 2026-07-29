# EngineeringDrawingPlugin

独立 SDK 插件「工程图」（`com.cloudsim.drawing`）：B-rep → OCC HLR 多视图图幅，支持草图、标注、用户图层、SVG/DXF/PDF 导出。

## 构建

Visual Studio 生成 **EngineeringDrawingPlugin**（须同时 Debug|x64 与 Release|x64）。产物：

- Debug：`bin/x64d/plugins/com.cloudsim.drawing/`
- Release：`bin/x64/plugins/com.cloudsim.drawing/`

依赖 Host ABI **≥ 1.42.0**（自定义剖切平面字段、`projectBrepToEngineeringDrawing`、模式条 Ribbon）。

## 使用

1. 打开含 B-rep 的文档  
2. 工作区切换进入 **工程图**（菜单栏下出现视图/绘图/标注 Ribbon）  
3. 左侧选择模型 → **生成图纸**（可选第一/第三角、轴测、剖视）  
4. Ribbon 绘制/标注；页工具栏可调局部放大倍率；导出 SVG/DXF/PDF
5. 返回三维场景

## 持久化

工程侧车 JSON `engineeringDrawing` **version 5**（views / dimensions / notes / paper / sketch / layers）。
