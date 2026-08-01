# ACCEPTANCE：3D 视口线框显示模式

## 子任务完成

| 任务 | 状态 | 说明 |
|------|------|------|
| T1 `applyBrepViewportWireframe` | 完成 | `BrepBackendVisual.h/.cpp` |
| T2 `setWireframeMode` + upsert | 完成 | 按分支分发；加载后尊重模式 |
| T3 颜色控制器 | 完成 | `brepWireOverlay` / `brepViewportWireframe` 压暗 |
| T4 文档 + 编译 | 完成 | BackendVisual / Widget GUIDE；Debug+Release 通过 |

## 验收对照 CONSENSUS

| 标准 | 结果 |
|------|------|
| BRep 开线框：拓扑边，无三角线/填充 | 实现：隐藏 fill + Phase2 边 Geode |
| BRep 关线框：恢复填充，移除 viewport 线框 | 实现 |
| Mesh 开/关：PolygonMode LINE/FILL | 实现（按 outer 分支） |
| 线框开时新导入 BRep | upsert / 点云 / 机器人挂载后补应用 |
| Debug\|x64 + Release\|x64 | `BackendVisual`、`CloudSimHost` 均通过 |

## 手工点验建议

1. 导入 STEP/BRep → 点视口线框：应见清晰拓扑边，无密三角网
2. 再关线框：恢复着色实体
3. 导入 Mesh（如 STL）→ 线框：仍为三角线框
4. 先开线框再导入 BRep：上屏即为拓扑边
