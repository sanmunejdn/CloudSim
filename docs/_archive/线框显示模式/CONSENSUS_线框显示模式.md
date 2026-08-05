# CONSENSUS：3D 视口线框显示模式（BRep 拓扑边）

## 1. 需求描述

视口工具栏「线框模式」对 **BRep 后端** 显示 **拓扑边（Edge）折线**，隐藏三角填充；对 **非 BRep** 仍用三角网格 `PolygonMode::LINE`。与导入参数 `showWireOutline` **解耦**（视口级模式）。

## 2. 已确认决策

| 项 | 结论 |
|----|------|
| Q1 BRep 填充 | **A**：隐藏填充，只画拓扑边 |
| Q2 非 BRep | **A**：保持 `PolygonMode::LINE` |
| Q3 与 `showWireOutline` | **A**：视口级模式，解耦 |
| Q4 装配 | 只对实际挂载的 visual 根施加一次（默认同意） |

## 3. 技术方案（摘要）

- 去掉对 `m_root` 的全局 `PolygonMode` OVERRIDE
- 按 `m_backendObjectRoots` 逐分支：
  - BRep（`BackendIdUserData::hasBrepShape`）：隐藏 fill Geode；`ensureBrepImportPickArtifacts` + 挂/显 `brepViewportWireframe`（复用 `edgePolylines`）
  - 非 BRep：在该 outer 的 StateSet 上设 `PolygonMode` LINE/FILL
- 线框已开时 `upsert`/`load` 成功后对新建分支补一次应用
- 颜色控制器对 `brepWireOverlay` / `brepViewportWireframe` 与 mesh 线框同样压暗

## 4. 验收标准

1. BRep 开线框：可见拓扑边，不可见三角网格线/填充面
2. BRep 关线框：恢复着色填充；视口临时线框节点移除
3. Mesh 开/关线框：行为与改前一致（三角 LINE / FILL）
4. 线框开着导入新 BRep：上屏即为拓扑边线框
5. Debug|x64 与 Release|x64 相关工程编译通过

## 5. 边界

- 不改 Phase2 离散算法
- 不新增「着色+边」独立按钮
- 不强制重建全部分支
