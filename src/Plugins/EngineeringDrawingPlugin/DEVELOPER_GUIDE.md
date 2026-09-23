# EngineeringDrawingPlugin 开发指南

## 1. 定位

| 项 | 说明 |
|----|------|
| 解决方案 | `CloudSim.sln` |
| 产物 | `bin/*/plugins/com.cloudsim.drawing/` |
| 职责 | B-rep → OCC HLR 工程图：多视图、标注、图层、SVG/DXF/PDF 导出 |

## 2. 边界

- Host ABI ≥ 1.42.0
- 用户帮助：`CloudSim/help/{zh,en}/drawing.html`
- 专题：[工程图](../../features/工程图/README.md)

## 3. 已交付能力

见 [README.md](README.md)。验证：Debug|x64 + Release|x64。
