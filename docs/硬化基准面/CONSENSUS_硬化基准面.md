# CONSENSUS — 硬化 + 基准面入门

## 需求描述

在「草图标配 + 扫描硬化」之上，硬化 Convert / UpToVertex / Draft / Offset / 多边形，并交付用户基准面试点，提升日常 Part 建模闭环。

## 验收标准

| # | 功能 | 验收 |
|---|------|------|
| 1 | Convert 真面边界 | 点选面 → 仅该面边界边投影入草图（含内环若有） |
| 2 | UpToVertex | 点选边 → 取靠近点击位置的端点为终止点 |
| 3 | Draft 中性面 | 侧栏可点选平面面；预览/提交用所选平面；未选时回退世界 XY |
| 4 | Offset | 外环 + 孔环均可偏置；结果自交则拒绝并提示 |
| 5 | 多边形 | 激活前可设边数 3–24（默认 6） |
| 6 | DatumPlane | Ribbon/命令创建（等距面、三点）；树可见；双击在该平面开草图 |

## 技术方案

- Convert / Vertex hit：Host ABI bump → `0x00012800`（1.40.0）
  - `PluginGeometryStepRef.hasHitPoint` + `hitWorldMm`
  - `discretizeBackendFaceEdgesToPolylines(faceRef, …)`
- Draft / Offset / Polygon / DatumPlane：插件 + FeatureDocument；Datum 不写入 Parametric tip

## 技术约束

- 注释纯中文、少而聚焦 Why；外科手术式改动
- 宿主与插件同版本，否则插件拒绝加载

## 边界

不做：成角基准面、基准轴/坐标系、特征级阵列、PipeShell 真扭转律、椭圆 GCS
