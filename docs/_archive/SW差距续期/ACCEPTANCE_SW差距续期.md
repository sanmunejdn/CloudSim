# ACCEPTANCE — SW差距续期

| 任务 | 状态 | 验证 |
|------|------|------|
| T1 椭圆/多边形/槽口 | 完成 | Ribbon 可画；导出可 Pad |
| T2 Offset | 完成 | 等距对话框 + 最大外环偏置 |
| T3 Convert | 完成 | 转换实体点选面 → 共面边投影 |
| T4 扫描扭转+边路径 | 完成 | twistDeg；模型边路径 |
| T5 到顶点/到面偏移 | 完成 | 枚举+UI+resolve+JSON |
| T6 Draft | 完成 | 拔模特征全栈 |

编译（Debug\|x64）：GeometryAlgorithm / Data / CloudSimHost / GeometricModelingPlugin 通过。

宿主 ABI：`0x00012700`（1.39.0）。
