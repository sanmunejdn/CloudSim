# ALIGNMENT — 特征阵列与拉伸深化

## 上下文

几何建模优先栈第 3 项。参考 `docs/几何建模/ROADMAP.md`。圆周阵列与成角基准面主干已存在；本期硬化 tip 贡献、铰链边向，并补 startOffset/双向拉伸。

## 边界

| 做 | 不做 |
|----|------|
| tipBefore + Cut 特征贡献 seed | 完整 TopoNaming 引擎 |
| 成角铰链用边方向 | 关联驱动基准面 |
| startOffset + TwoDirections | PipeShell/引导线 |
| 文档纠偏 + TopoNaming ALIGNMENT 草案 | 包 D 视口点选/Suppress 全量 |

## 约束

新 Host 字段 bump ABI 1.48.0；Debug+Release 双编。
