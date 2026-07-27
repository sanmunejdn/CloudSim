# ACCEPTANCE — 实体扫描（Sweep）特征 MVP + 完善

## 范围

实体扫描凸台 / 扫描切除：闭合轮廓草图 + 开放路径草图（Line/Arc）→ `BRepOffsetAPI_MakePipe` → Fuse/Cut 进 Parametric Body。

## 验收清单

| # | 场景 | 期望 | 结果 |
|---|------|------|------|
| 1 | 路径草图（折线或线+弧）+ 闭合矩形轮廓 → 扫描凸台 | 场景可见实体；特征树有 Sweep | ✅ 自测 Sweep rebuild 非空；GUI 建议再确认 |
| 2 | 同路径/轮廓 → 扫描切除切穿已有 Pad | 切除生效；特征树有 SweepCut | ✅ 自测 SweepCut rebuild；GUI 建议再确认 |
| 3 | 改路径或轮廓草图结束编辑 | 下游 Sweep/SweepCut 重建 | ✅ 代码路径覆盖；GUI 建议再确认 |
| 4 | Undo | 恢复 history；JSON 含 `Sweep`/`SweepCut` + `pathSketchRefId` | ✅ 自测 JSON 断言 pathSketchRefId |
| 5 | Debug 编译 | Algo → Data → SDK → CloudSimHost → GeometricModelingPlugin 通过 | ✅ |

## 完善项

| 项 | 结果 |
|----|------|
| 中文 UI 无乱码 | ✅ `\u` 转义 |
| MakePipe 失败可见 | ✅ preview 返回 err + Host logWarn |
| 路径真弧边 | ✅ `pathSegments` + `GC_MakeArcOfCircle` |
| 闭环路径拒绝 | ✅ export 层 |
| 改草图下游重烤 pathSegments | ✅ |
| 侧栏失败状态行 | ✅ |
| 草图下拉启发式 | ✅ |

## 不做（本期）

开曲面扫描、引导线、扭转、模型边路径、多轮廓孔岛、薄壁扫描。

## 版本

- Host / SDK：`1.31.1`（`0x00011F01`）
- 插件 `minHostVersion`：`1.31.1`

失败模式备忘：[`QA_扫描特征_失败模式.md`](QA_扫描特征_失败模式.md)
