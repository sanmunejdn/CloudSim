# 特征表跨工件叠加 — 验收记录

## 需求与决策

- 切换工件不清特征表，已选特征保留，新工件拾取继续追加。
- 合并轨迹落在**首特征工件**的模型系，下游预览/编辑/出程序不变。
- 桌面、AI、Headless、Web 均支持；表格全部显示，「工件」列标明归属。

## 实现

| 层 | 变更 |
|----|------|
| GeometryAlgorithm | `FeatureEntry.workpiece`（空 = 回落 `doc.workpiece`）；`featureEntryWorkpiece()`；JSON 读写 `features[].workpiece` |
| GeometryServices | `discretizeFeatureListAcrossWorkpieces`：按表内连续同工件分段离散，非首工件经 `inv(W_首)·W_i` 换算，段间插 `segmentEndExclusive`，输出补齐每行工件的 resolved doc |
| RobotWidget | 切 combo 不 `clearAll`；拾取写行工件；跨工件「追加到选中」拒绝；`FeatureTableModel::ColWorkpiece`；BREP 信息显示工件；离散改用新接口 |
| AI | 计划读 `features[].workpiece`；非 `revise` 按 (工件, featureId) 合并叠加，`revise` 整表替换；revise 导出每行工件 |
| Headless | `setFeaturesAndDiscretize` 改用新接口 |
| Web | `TrajFeature.workpiece`；拾取打标、跨工件追加拦截、离散带每行工件、表格「工件」列 |

## 验证

| 项 | 结果 |
|----|------|
| Debug\|x64：GeometryAlgorithm / GeometryServices / RobotScene / RobotWidget / CloudSimHostCore / CloudSimHostHeadless / CloudSimHost | 通过 |
| Release\|x64：同上 | 通过 |
| Web `build:debug` / `build:release`（含 tsc） | 通过，产物 `bin\x64d\web` / `bin\x64\web` |
| SelfTest `featureListEntryWorkpieceJson` | **未执行**：既有缺陷 #92（自测进程未注入 ConfigRegistry，在其前崩溃） |
| 桌面端到端（两零件各拾取 → 离散 → 预览） | 待用户点验 |

## 已知限制

- 合并轨迹是**离散时刻**的刚性快照：离散后单独移动非首工件，其轨迹不跟随（需重新离散）。
