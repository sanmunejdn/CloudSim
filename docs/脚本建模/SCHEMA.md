# SCHEMA — 脚本建模格式速查

两种根格式由 `ScriptModelIo` 自动判别。本文覆盖 **history 全特征**、**草图实体/约束**、**feature.compose 全 API**。

---

## 1. History（`.cloudsim-part.json`）

根对象：`{ "seq": int, "features": [ Feature, ... ] }`  
也可包一层 `{ "parametricHistory": { ... } }`。

### 1.1 Feature 公共字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `id` / `name` | string | 特征 id / 显示名 |
| `kind` | string | 见下表 |
| `suppressed` / `visible` | bool | 默认 false / true |
| `sketchRefId` | string | Pad/Pocket/Revolve 等引用的草图 id |
| `plane` | object | 草图平面：`origin`/`axisX`/`axisY`/`normal`/`isPlanar` |

### 1.2 `kind` 一览（Parametric tip）

| kind | 关键字段 |
|------|----------|
| `Sketch` | `profile`（世界 xyz 闭合折线）、可选 `sketchDocument`、`profileHoles` |
| `Pad` / `Pocket` | `lengthMm`、`length2Mm`、`startOffsetMm`、`endCondition`、`reversed`、`draftAngleDeg`；到面/到点见下 |
| `Sweep` / `SweepCut` | `pathSketchRefId` 或 `path` / `pathSegments`、`twistDeg` |
| `Fillet` | `edgeIndices[]`、`radiusMm` |
| `Chamfer` | `edgeIndices[]`、`chamferDistMm` |
| `Revolve` / `RevolveCut` | `revolveAngleDeg`、`axisO`/`axisD` |
| `LinearPattern` | `patternCount`、`patternD`[dx,dy,dz]、可选 `patternSourceFeatureId` |
| `CircularPattern` | `patternCount`、`patternAngleDeg`、`axisO`/`axisD`、可选 `patternSourceFeatureId` |
| `Mirror3D` | `mirrorPlane`、`mirrorKeepOriginal` |
| `Loft` / `LoftCut` | `sketchRefId` + `loftSketchRefId` |
| `Shell` | `faceIndices[]`、`shellThicknessMm` |
| `Draft` | `faceIndices[]`、`draftAngleDeg`、中性面可用 `mirrorPlane` 同类平面字段（见实现） |

`endCondition`：`Blind` | `UpToFace` | `MidPlane` | `ThroughAll` | `UpToVertex` | `OffsetFromFace` | `TwoDirections`  
UpToFace：`upToFacePlane` / `upToFaceBackendId` / `upToFaceIndex`  
UpToVertex：`upToVertex`[x,y,z]、`upToVertexIndex`  
OffsetFromFace：`offsetFromFaceMm`  
TwoDirections：`lengthMm` + `length2Mm`；Blind 可用 `startOffsetMm`

**DatumPlane / DatumPlaneAngle**：仅 FeatureDocument 侧车，不进 Parametric history。

### 1.3 `pathSegments.kind`

| 值 | 含义 |
|----|------|
| 0 | Line：`a`→`b` |
| 1 | Arc：`a`/`b`/`m` |
| 2 | SplineThrough |

---

## 2. 草图 `sketchDocument`（嵌入 Sketch 特征）

可为 **对象** 或 **JSON 字符串**。字段与 `SketchDocument2d` 一致：

| 数组 | 元素字段 |
|------|----------|
| `points` | `id`,`u`,`v`,`fixed` |
| `lines` | `id`,`p1`,`p2`,`construction` |
| `arcs` | `id`,`pStart`,`pMid`,`pEnd`,`construction` |
| `circles` | `id`,`center`,`radius`,`construction` |
| `ellipses` | `id`,`center`,`majorR`,`minorR`,`angleRad`,`construction` |
| `splines` | `id`,`points`（过点 id）、可选 `controlPoints`、`mode`（0 过点 / 1 控制点）、`construction` |
| `constraints` | `kind`(int)、`a`,`b`,`value`、可选 `c`（对称轴） |

### 2.1 约束 `kind` 枚举（int）

| 值 | 含义 |
|----|------|
| 0 | Coincident |
| 1 | Horizontal |
| 2 | Vertical |
| 3 | EqualLength |
| 4 | Distance |
| 5 | Parallel |
| 6 | Perpendicular |
| 7 | Radius（圆） |
| 8 | Angle |
| 9 | ArcRadius |
| 10 | Tangent |
| 11 | Symmetric（`c`=轴直线） |
| 12 | Midpoint |
| 13 | MajorRadius（椭圆） |
| 14 | MinorRadius（椭圆） |

出体仍依赖 Sketch 上的 `profile` 世界折线（与 UI 结束草图时烤坐标一致）。

---

## 3. Compose（`.cloudsim-compose.json`）

```json
{ "version": 2, "domain": "feature.compose", "steps": [ { "id", "api", "args" } ] }
```

| api | 要点 |
|-----|------|
| `askClarify` | `questions[]` |
| `extrudeSketchProfileToBrep` | pad/pocket；profile helpers 或 `profile_xyz_mm`；`end_condition`；pocket 需 `target` |
| `filletEdgesToBrep` | `radius_mm`；`edge_indices` 或 `edges` |
| `chamferEdgesToBrep` | `distance_mm`；同上 |
| `revolveSketchProfileToBrep` | boss/cut；`angle_deg`；轴 |
| `linearPatternBodyToBrep` | `count`,`dx_mm`…；可选 `source_feature_id` |
| `circularPatternBodyToBrep` | `count`,`angle_deg`,`axis_*` |
| `sweepSketchProfileToBrep` | path helpers；`twist_deg` |
| `loftSketchProfilesToBrep` | 两截面 |
| `shellFacesToBrep` | **必填** `face_indices` |
| `draftFacesToBrep` | **必填** `face_indices` |

Compose **无** Mirror3D / TwoDirections / startOffset → 用 history JSON。

---

## 4. Python `cloudsim_geom`

```python
import cloudsim_geom as g
g.list_bodies()
g.export_history(body_id=None)      # -> str
g.import_history(json_str, body_id=None)
g.run_compose(json_str)             # -> summary str
```

在几何模式 Ribbon「Python」控制台中执行；与磁盘 JSON 字符串互通。
