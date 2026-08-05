# 脚本模板使用说明

完整字段表见上级 [`SCHEMA.md`](../SCHEMA.md)。

## 文件一览

| 文件 | 用途 |
|------|------|
| `01_sketch_full.history.json` | **可导入**：草图实体+约束全覆盖 + Pad |
| `02_features_catalog.history.json` | 全 `kind` 字段目录；多数示范特征 `suppressed` |
| `03_all_compose_apis.compose.json` | Compose 全 API；shell/draft 需改 `face_indices` |
| `model_history.py` | Python 拼 history → `import_history` |
| `model_compose.py` | Python 拼 compose → `run_compose`（默认去掉 shell/draft） |

## Ribbon

1. **导入新建 / 导入替换**：选 `01_*.history.json` 或自改 catalog  
2. **运行 Compose**：选 `03_*.compose.json`  
3. **Python**：粘贴 `model_*.py` 后 Run  

## 草图命令 ↔ JSON

| UI | sketchDocument |
|----|----------------|
| 直线 | `lines[]` |
| 构造线 | `lines[].construction=true` |
| 圆弧 | `arcs[]` |
| 圆 | `circles[]` |
| 椭圆 | `ellipses[]` |
| 样条过点/控制点 | `splines[]` + `mode` 0/1 |
| 尺寸/几何约束 | `constraints[].kind` 0–14（见 SCHEMA） |

出体必须同时提供 Sketch.`profile` 世界折线。

## Compose vs History

| 能力 | Compose | History |
|------|---------|---------|
| Pad/Pocket/Fillet/… | 有 | 有 |
| Mirror3D | 无 | 有 |
| TwoDirections / startOffset | 无 | 有 |
| 精细草图约束 | 弱（helpers） | 完整 `sketchDocument` |
