# FINAL — 脚本建模

## 摘要

在现有 parametric history 与 `feature.compose` 上增加磁盘导入/导出与进程内 Python，无第三套 DSL、无 ABI bump。

## 交付物

| 路径 | 内容 |
|------|------|
| `docs/脚本建模/` | 6A + examples + **SCHEMA** + **templates** |
| `ScriptModelIo.*` | JSON 判别 |
| Ribbon + Plugin handlers | 导出/导入/Compose |
| `CloudSimGeomPython.*` | `cloudsim_geom` + 控制台 |

## 编译

`GeometricModelingPlugin` Debug|x64 + Release|x64：**通过**（产物 `bin\x64d\plugins\com.cloudsim.geomodeling\` / `bin\x64\plugins\...`）。
