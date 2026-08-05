# CONSENSUS — 网页端轨迹对等

## 需求
完整对等桌面「轨迹生成 / 轨迹编辑」：视口射线 BREP 拾取线/面、CAD+Mesh、PathPlan、全量算子、模板、撤销重做、Raw 预览与 Apply 出 LINE。

## 验收（分阶段）
1. 可创建/绑定 PathPlan，开始修改门闩有效 — **通过**
2. 视口点选边/面 → 特征表 → 离散 → Raw 折线 — **通过**
3. Mesh 子页可生成 Raw — **通过**（截面；B样条需三角索引）
4. 编辑页配方/管线预览/应用出 LINE — **通过**
5. 全算子 + 模板 + undo — **通过**
6. API/TODO 文档同步 — **通过**

## 约束
桌面零回归；Headless 旁路；Debug+Release 双编；public-fallback 前端。

## 技术方案要点
- Host：`HeadlessTrajectorySession`（无 OSG）
- Gateway：`/api/trajectory/*` + `/api/pick/mesh-element` + `/api/robot/path-plan`
- 前端：右坞「轨迹生成/编辑」；指令树选中 PathPlan 自动 bind
