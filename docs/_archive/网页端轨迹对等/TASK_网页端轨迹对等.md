# TASK — 网页端轨迹对等

## T1 骨架
- 输入：DocumentHost headless
- 输出：`HeadlessTrajectorySession` + PathPlan API + 右坞双页签 + 开始修改门闩
- 验收：可创建/绑定；未 begin-edit 不可改特征

## T2 特征拾取
- 输入：世界系射线 mm
- 输出：`/api/pick/mesh-element` + 特征表 + 离散 + Raw 折线
- 验收：点选边/面 → 表有行 → 青色 Raw

## T3 Mesh
- 输入：MeshTrajectorySpec
- 输出：同一 PathPlan Raw
- 验收：截面可生成；B样条需三角索引

## T4 编辑管线
- 输入：Raw + ops
- 输出：preview / apply LINE
- 验收：配方填充 → 预览变化 → 应用出组

## T5 全量算子与模板
- 输出：20 种算子入队；AppData 模板；草稿 undo/redo
- 验收：存取模板；撤销可用

## T6 文档与指令树
- 输出：选中 PathPlan 自动 bind；API/TODO/6A 收尾
