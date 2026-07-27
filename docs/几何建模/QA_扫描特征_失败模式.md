# Sweep 手工验收 — 已知失败模式（完善期）

在 GUI 全链路手测前，代码路径已覆盖下列失败与拦截点；自测通过后勾选 ACCEPTANCE #1–#4 中可由自动化覆盖的部分，余下依赖场景手测。

## 预检（插件）

| 条件 | 表现 |
|------|------|
| 草图数 < 2 | 拒绝打开扫描侧栏 |
| 轮廓 id == 路径 id | 清除预览并告警 |
| Cut 且无活动 Body | 拒绝 |
| 轮廓未闭合 / 导出失败 | hostLogWarn，无 staging |
| 路径分叉、断链、空 | exportOpenPath 失败文案 |
| 路径首尾闭合（闭环） | 导出拒绝（须 open wire） |

## Algo / Host

| 条件 | errMsg / 行为 |
|------|----------------|
| profile < 3 点 | `profile needs >=3 points` |
| path < 2 点 | `path needs >=2 points` |
| 边无法成 wire | `path edges do not form a wire` |
| MakePipe 失败 | `MakePipe failed`（预览 logWarn + 清 staging） |
| SweepCut 无 tip | `SweepCut requires base solid` |
| mesh 离散失败 | 预览失败可见 |

## 验收映射

- #1 Boss：自测构造矩形轮廓 + 折线路径 rebuild 非空
- #2 Cut：自测 Pad 后再 SweepCut
- #3 下游重建：改草图 profile 后 push history（代码路径已覆盖）
- #4 Undo / JSON：自测 history 含 Sweep + pathSketchRefId
