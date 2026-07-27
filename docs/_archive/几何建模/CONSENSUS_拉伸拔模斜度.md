# CONSENSUS — 拉伸/切除拔模斜度

## 需求

Pad（拉伸）与 Pocket（切除）增加拔模斜度参数，默认 0°。

## 方案

- 字段名 `draftAngleDeg`（度），缺省 JSON 为 0。
- 定长：`MakePrism` 后对侧壁 `BRepOffsetAPI_DraftAngle`（失败则保留棱柱）。
- 对称（MidPlane）：正/反半棱柱**各自**拔模后再 Fuse（整块跨中性面拔模无效）。
- 贯通：UI → FeatureDocument → PluginSketchExtrudeParams → Host → ParametricFeature → SketchExtrude。
- Host ABI **1.32.0**（params POD 增字段）。

## 验收

1. 侧栏有拔模角度，改值刷新预览。
2. 确认后特征持久化，重开/重建保留斜度。
3. 编辑 Pad/Pocket 可改拔模。
4. 0° 行为与旧版一致。
5. 对称 + 拔模：体积相对无拔模变化，形体关于草图面对称。
