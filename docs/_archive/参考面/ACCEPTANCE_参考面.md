# ACCEPTANCE — 参考面（C + D + D-S1）

| 项 | 状态 | 说明 |
|----|------|------|
| T1 文档 D-S1 | 完成 | CONSENSUS/TASK 已锁定 |
| T2 FeatureDocument 字段 | 完成 | 源引用 + `datumPlaneId` + JSON |
| T3 Host ABI 1.49.0 | 完成 | `pickSketchSupportPlane`；OsgWidget extra 命中 |
| T4 P1 接线 | 完成 | 等距可选基面/模型面；新建草图可点选 Datum |
| T5 P2 侧栏+跟随 | 完成 | 参数页 `datum.offset`/`datum.angle`；reevaluate；草图跟随 |
| T6 编译 | 完成 | CloudSimHost + GeometricModelingPlugin Debug\|x64 & Release\|x64 |

## 手工验收建议

1. 基准面 → 等距 → 点选 XY → 偏移 20 → 树出现 Datum，视口绿框  
2. 新建草图 → 点选该参考面 → 进入草图  
3. 双击 Datum → 改 `datum.offset` → overlay 与挂接草图平面更新  
4. 宿主与插件须同为 **1.49.0**（`0x00013100`）
