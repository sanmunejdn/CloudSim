# CONSENSUS — design.calc（Wave A/B/C + feature.compose）

## 已确认决策

1. **公式可写入仓库**：自原表改写为可测 Python（`tools/design-calc`），**不**提交原 `.xls`。
2. **范围**：Wave **A + B + C**（电机/减速/惯量转矩 + 齿轮几何材料 + 蜗杆/键/带链基础）。
3. **CAD 对接**：计算输出同时生成 **`feature.compose` ActionPlan** + **模板 JSON**（齿轮/齿条/蜗轮现阶段为**毛坯回转体/拉伸体**占位；精确渐开线齿廓属后续 Host 能力）。

## 验收标准

| ID | 标准 |
|----|------|
| A1 | `motor.power` / `reducer.rated_power` / `motor.y_series_lookup` 单测通过，样例与源表一致 |
| A2 | `load.torque`（丝杠子集）与 `inertia.shape`（圆柱/圆盘）可算 |
| B1 | `gear.spur_helical_shift` / `gear.rack` / `gear.high_shift_dims`（核心量）输出 SI(mm) |
| B2 | `gear.materials` 可按牌号查表 |
| C1 | `worm.geometry` / `key.strength` / `chain.sprocket_pitch` / `belt.v_length` 可用 |
| F1 | `to_feature_compose`：齿轮副/齿条/蜗杆毛坯 → 合法 `domain=feature.compose` JSON |
| F2 | 模板目录含可填参的 compose 样例 |

## 非目标（本轮）

- Wave D 大表（锥齿/齿条全设计程序、伺服选型全自动）完整移植  
- 渐开线齿面造型 Host API  
- 运行时嵌入 CloudSim exe（本轮交付离线工具链 + 契约；AI Domain 接线可二期）
