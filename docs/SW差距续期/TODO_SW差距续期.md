# TODO — SW差距续期

## 使用注意

1. 宿主与插件须同为 **1.39.0+**（`0x00012700`），否则插件拒绝加载。
2. 重新编译并部署：`CloudSimHost.dll` + `GeometricModelingPlugin.dll` + `GeometryAlgorithm.dll` + `Data.dll` + `CloudSimPluginSDK.dll`。

## 已知 MVP 债

| 项 | 说明 | 建议 |
|----|------|------|
| 椭圆约束 | 未进 PlaneGCS | 后续加椭圆半径/角度约束 |
| 扫描扭转 | 轮廓绕起点切向预转 | PipeShell 扭转律 |

> 多边形边数 / Convert / Offset / 到顶点 / Draft 中性面 / 用户基准面 → 见 `docs/硬化基准面/`（已交付 1.40.0）

## 未选加包（仍可开）

- B 特征级线性阵列（非 tip 整实体）
- DatumPlane 文档持久化与视口 overlay
