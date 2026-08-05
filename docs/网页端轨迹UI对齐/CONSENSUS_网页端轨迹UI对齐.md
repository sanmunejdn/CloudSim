# CONSENSUS — 网页端轨迹 UI 对齐

## 决策确认（用户 2026-08-04）

| 项 | 选择 |
|----|------|
| 范围 | 仅 CAD/BREP 轨迹生成页 |
| 参数变更 | ~400ms 防抖自动再离散 |
| 加分 | 右键/按钮删除特征内面/边索引 |
| 风格 | 尽量仿桌面分区与控件密度 |

## 验收标准

1. 特征表 5 列：`#` / `特征ID` / `离散策略`(中文) / `几何摘要` / `状态`
2. `GET /api/trajectory/feature-schema?strategyId=` 返回合并字段 + defaults + displayNameZh
3. 选中特征后显示策略参数面板（FaceParamSurface 全字段可用）
4. 改参数 → 写入当前特征 → 400ms 后自动 `discretize`
5. 离散模板存/载/删（`templates/discretize`）
6. 可删除特征内单个 face/edge 索引
7. 拾取状态文案对齐桌面（如「3D 拾取未激活」）
8. 桌面零回归；Debug+Release 双编；部署 public-fallback

## 技术要点

- Host 调 `featureDiscretizerAllParamFields` / `DisplayNameZh` / `DefaultParams`
- 前端仿桌面分区：工件 → 特征表 → 拾取按钮行 → 状态 → 策略+参数+模板
- 不改 Mesh / 轨迹编辑页（本轮）
