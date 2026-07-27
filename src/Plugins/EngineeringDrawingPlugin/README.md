# EngineeringDrawingPlugin

独立 SDK 插件「工程图」：从文档 B-rep 生成第一角法三视图（OCC HLR），中央二维图幅画布查看。

## 构建

Visual Studio 生成 **EngineeringDrawingPlugin**（须同时 Debug|x64 与 Release|x64）。产物：

`bin/x64d/plugins/com.cloudsim.drawing/EngineeringDrawingPlugin.dll` + `plugin.json`（Debug）

依赖 Host ABI **≥ 1.33.0**（`projectBrepHlrToDrawing`）。

## 使用

1. 打开含 B-rep 的文档  
2. 菜单 **工程图 → 进入工程图**  
3. 左侧选择模型 → **生成三视图**  
4. **返回三维场景**
