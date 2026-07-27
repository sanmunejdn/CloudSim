# TODO — 几何建模 SDK 插件

## 待办

1. **相机射线落面**：已实现 `OsgWidget::intersectScreenWithPlaneMm` + Host `mapScreenToSketchPlane`  
2. **交互绘线工具**：插件内 `SketchEditSession` + Line/Arc/Circle/Rect + 捕捉（端点/中点/网格/正交）；OneCAD port 仍作参考未整仓编译  
3. **特征改参 Undo**：Pad 长度变更写入 CommandStack 快照  
4. **Pocket 基实体选择**：侧栏「切除目标实体」下拉 = `activeBodyId`；Pocket 禁用新建实体  
5. **清理克隆缓存**：可选删除 `third_party/planegcs_src`、`onecad_src` 整仓（已复制所需文件）  
6. **LGPL 合规发行**：提供 planegcs 目标文件替换说明  
7. **编译**：插件工程使用 **C++20**（PlaneGCS 依赖 `std::numbers` / `std::ranges`）；Host/其余仍为 C++17  
8. **SDK 输出路径**：确认运行时 `bin/x64d/CloudSimPluginSDK.dll` 与插件旁 `plugin.json` 已部署  

## 缺少的配置

- 无额外 `.env`  
- 需本机已配置 `bin/SDK/eigen`、`boost_1_79_0`、OCC（Host/GeometryAlgorithm 已有）  
