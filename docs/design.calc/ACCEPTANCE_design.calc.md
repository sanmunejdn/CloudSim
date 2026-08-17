# ACCEPTANCE — design.calc Wave A/B/C

## 决策落地

| 项 | 状态 |
|----|------|
| 公式写入仓库（不交原 xls） | 已落地 `tools/design-calc` |
| Wave A/B/C | 已实现核心 API + 单测 |
| 接 feature.compose / 模板 | `to_feature_compose` + `templates/*_blank.compose.json` |

## 测试

```text
cd CloudSim/tools/design-calc
python -m unittest tests.test_waves -v
→ 16 tests OK（2026-08-17）
```

| ID | 结果 |
|----|------|
| A1 motor/reducer/Y 查表 | PASS（与源表样例对齐） |
| A2 load/inertia | PASS |
| B1 gear/rack/high_shift | PASS |
| B2 materials | PASS（`gear_materials_clean.csv`） |
| C1 worm/key/belt/chain | PASS |
| F1 compose 映射 | PASS |
| F2 模板文件 | 已生成 gear/rack/worm |

## 已知限制

- 齿轮/蜗杆 compose 为**毛坯**，无渐开线齿廓  
- 变位总系数 ξΣ 在 A≠A0 时用工程近似（`xi_sum_approx`）  
- Wave D（锥齿全程序、伺服全自动）未迁  
- 尚未注册 CloudSim 运行时 Domain `design.calc`（离线 CLI / 手工跑 compose）

## TODO（产品接线）

1. AI Domain `design.calc`：口语 → 调本库 → 确认面板 → `executeActionPlan`  
2. Host 齿轮特征（可选）替换毛坯  
3. 装配中心距 A 写入场景位姿
