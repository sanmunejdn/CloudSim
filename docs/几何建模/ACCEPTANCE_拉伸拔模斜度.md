# ACCEPTANCE — 拉伸/切除拔模斜度

## 实现完成

| 项 | 状态 |
|---|---|
| `SketchExtrudeParams::draftAngleDeg` + `BRepOffsetAPI_DraftAngle` | 已完成 |
| MidPlane（对称/双向）半棱柱分别拔模再 Fuse | 已完成（点验修复） |
| `ParametricFeature` / JSON 持久化 | 已完成 |
| `PluginSketchExtrudeParams` + Host 1.32.0 | 已完成 |
| 侧栏拔模控件（Pad/Pocket） | 已完成 |
| FeatureDocument 读写 / 编辑回填 | 已完成 |

## 编译与点验

- GeometryAlgorithm / Data / CloudSimPluginSDK / CloudSimHost / GeometricModelingPlugin：通过
- 离线：`bin\x64d\SketchExtrudeDraftVerify.exe`
  - Blind+draft5：vol≈29282（相对 32000 减小）
  - MidPlane：vol=32000，Z∈[-10,10]
  - MidPlane+draft5：vol≈30621，delta≈-1379
- 新宿主运行时：`ParametricBrep Pad/Pocket/MidPlane+draft/Sweep/history self-test OK.`
- 插件加载：`com.cloudsim.geomodeling` OK（minHostVersion 1.32.0）

## 手工 UI（可选）

1. [ ] 侧栏选「对称」+ 改拔模，预览两端收分
2. [ ] 确认后保存/重开保留 `draftAngleDeg` 与 MidPlane
