# FINAL — 拉伸/切除拔模斜度

## 总结

Pad（拉伸）与 Pocket（切除）增加 `draftAngleDeg` 参数，算法在棱柱生成后对侧壁施加 `BRepOffsetAPI_DraftAngle`，全链路 UI → 特征 JSON → Host → 参数化重建已打通。Host ABI 升至 **1.32.0**。

## 关键改动

- 算法：`GeometryAlgorithm/SketchExtrude.*`
- 数据：`ParametricBrepFeature.*`、`ParametricBrepBackendData.cpp`
- SDK：`PluginGeometryTypes.h`、宿主版本宏、`plugin.json` `minHostVersion`
- Host：`PluginGeometryHostImpl.cpp`（预览/提交）
- 插件：侧栏 `m_draftAngle`、`FeatureDocument`、提交/编辑路径

## 质量

- 默认 0°，旧工程缺字段按 0 解析，兼容既有模型
- 拔模失败不崩；无侧壁可拔时保持棱柱
