# TODO — 硬化 + 基准面入门

## 部署

1. 宿主与插件须同为 **1.47.0+**（`0x00012F00`）。
2. 重新编译并部署：`CloudSimHost.dll` + `GeometricModelingPlugin.dll` + `GeometryAlgorithm.dll` + `CloudSimPluginSDK.dll` + `Data.dll`。

## 已知债

| 项 | 说明 |
|----|------|
| Offset | bevel 后仍自交则拒绝；极窄孔环可能失败 |

## 已关闭（P1+P2）

- Convert 圆边 → Arc/Circle；成角基准面；真 Vertex；Fillet 智能选边  
- Pattern tip 语义；圆周阵列；Revolve 可选轴；Offset 尖角 bevel  
- AI Sweep/Loft/Shell/Draft/CircularPattern（见 `../特征史AI/`）
