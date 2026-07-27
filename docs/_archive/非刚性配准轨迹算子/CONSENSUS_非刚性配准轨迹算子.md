# CONSENSUS — 非刚性配准轨迹算子

## 验收标准

| ID | 场景 | 期望 |
|----|------|------|
| T-NR1~4 | 源/目标 mesh/PC 四组合 | SPARE 成功则 scope 内点位随变形写回 |
| T-NR5 | 绑定点超距 | 跳过并计入 missCount |
| T-NR6 | BREP 或缺失 backend | validate/processPath 失败 |
| T-NR7 | Preview/Apply/Undo | 与其它块一致 |

## 技术方案

1. `NonRigidRegistration` 原子块 → `ctx.nonRigidTrajectoryWarp`
2. 源 mesh：三角面重心绑定；源 PC：最近点索引绑定
3. 对 snapshot 拷贝跑 SPARE，写回变形后位置

## 约束

Builtins 不链 Data.lib；SPARE 与绑定在 RobotScene 实现。
