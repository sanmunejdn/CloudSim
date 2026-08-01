# EngineeringDrawingPlugin

独立 SDK 插件「工程图」（`com.cloudsim.drawing`）：B-rep → OCC HLR 多视图图幅，支持草图、标注、图层/图块、SVG/DXF/PDF 导出。

用户操作说明见产品帮助：**帮助 → 帮助文档 → 工程图纸工作区**（`CloudSim/help/{zh,en}/drawing.html`，由 `CloudSim/help/_generate_manual.py` 生成）。

## 构建

Visual Studio 生成 **EngineeringDrawingPlugin**（须同时 Debug|x64 与 Release|x64）。产物：

- Debug：`bin/x64d/plugins/com.cloudsim.drawing/`
- Release：`bin/x64/plugins/com.cloudsim.drawing/`

依赖 Host ABI **≥ 1.42.0**（自定义剖切平面、`projectBrepToEngineeringDrawing`、模式条 Ribbon）。

## 使用

1. 打开含 B-rep 的文档  
2. 工作区切换进入 **工程图**（菜单栏下出现视图/绘图/标注 Ribbon）  
3. 左侧选择模型 → **生成图纸**（可选第一/第三角、轴测、剖视、半剖）  
4. Ribbon 绘制/标注/修改；右侧 **属性** 面板改图层与 ByLayer/ByBlock；导出 SVG/DXF/PDF  
5. 返回三维场景  

## 能力摘要（至 Q17）

- 剖切握柄联动、细节圆可编、半剖简版；钉住投影  
- 连续/基线尺寸；Break/Join/**Stretch**；草图阵列；线-弧倒圆/倒角  
- BYBLOCK 与属性上图；DXF 圆/弧/ATTDEF；粗糙度与 GD&T 轻量框  
- 捕捉与选择高亮  

后置项见 [`docs/工程图纸/TODO_工程图纸.md`](../../../docs/工程图纸/TODO_工程图纸.md)。

## 持久化

工程侧车 JSON `engineeringDrawing`（views / dimensions / notes（含 noteKind）/ paper / sketch / layers / blocks / hatches 等）。
