# ALIGNMENT：3D 视口线框显示模式（BRep 拓扑边）

## 1. 项目上下文

| 项 | 说明 |
|----|------|
| 入口 | `ViewportToolBar`「线框模式」→ `DocumentPage` → `OsgWidget::setWireframeMode` |
| 现状实现 | 对 `m_root` 全局 `osg::PolygonMode::LINE`，把所有三角面画成三角网格线框 |
| BRep 显示 | `BrepBackendVisual` 已有 `brepWireOverlay`（`edgePolylines` / Phase2），导入默认 `showWireOutline=false` |
| 文档约定 | `BackendVisual/DEVELOPER_GUIDE.md` 已写「用户开启线框时经 `ensureBrepImportPickArtifacts` 补 Phase2」，但工具栏未接线 |

## 2. 原始需求

优化 3D 页面工具栏线框显示：对有 BRep 后端的对象，应显示 **BRep 拓扑边线框**，而不是 OSG 三角网格的线框。

## 3. 边界确认

**范围内**

- 工具栏线框开关对 **BRep 后端** 的显示语义改造
- 复用已有 Phase2 `edgePolylines` / `buildBrepEdgeWireGeode` / `ensureBrepImportPickArtifacts`
- 非 BRep（Mesh 等）在未另定方案前保持现有三角线框行为，或明确并列策略
- 同步相关 DEVELOPER_GUIDE / 本任务文档

**范围外**

- 改 BRep 离散精度 / Phase2 算法本身
- 新增独立「着色+边」工具栏按钮（除非共识选择合并语义）
- 点云专用线框

## 4. 需求理解

当前线框 = 渲染态 `PolygonMode::LINE`，对 BRep 等同「看三角剖分」。期望线框 = CAD 语义的 **边（Edge）折线**，与拾取索引同源。

推荐实现方向（待确认）：

1. **关**：恢复填充；BRep 去掉/隐藏由线框模式挂上的 `brepWireOverlay`（导入时本来就无边叠加）
2. **开**：
   - BRep：不套三角 `LINE`；确保 Phase2；显示拓扑边；填充隐藏或保持（见决策点）
   - 非 BRep：继续对该分支或场景使用 `PolygonMode::LINE`（或等价）
3. 线框已开时新加载的 BRep：加载后立即按当前模式应用

## 5. 疑问澄清（需确认）

### Q1 — BRep 线框开启时填充面如何处理？（关键）

| 选项 | 含义 |
|------|------|
| **A（推荐）** | 隐藏填充，仅拓扑边（经典 CAD Wireframe） |
| B | 保留着色填充 + 叠加拓扑边（Shaded with Edges） |
| C | 填充半透明 + 拓扑边 |

### Q2 — 非 BRep（Mesh / 其它）线框行为？

| 选项 | 含义 |
|------|------|
| **A（推荐）** | 保持现状：三角 `PolygonMode::LINE` |
| B | 改为 feature-edge / `meshWireOverlay`（与导入 `showWireOutline` 一致） |
| C | 仅改 BRep；Mesh 线框按钮无效果 |

### Q3 — 与导入参数 `showWireOutline` 的关系？

| 选项 | 含义 |
|------|------|
| **A（推荐）** | 工具栏线框是**视口级模式**；与单对象导入时的 `showWireOutline` 解耦。关模式时不强制清掉对象自带边叠加（若将来有） |
| B | 开线框 = 全场景 `showWireOutline=true` 并重建所有分支 |

### Q4 — 装配 STEP（多零件共享父 visual）？

父节点一条 OSG 分支 + 子零件 `pickVisualAlias`。线框应对**实际挂载的 visual 根**（通常 importParent）施加 BRep 边，子 alias 不重复画边。是否按此理解？**默认：是**
