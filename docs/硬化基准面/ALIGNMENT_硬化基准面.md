# ALIGNMENT — 硬化 + 基准面入门

## 项目上下文

- 栈：GeometryAlgorithm → ParametricBrep → PluginGeometryHost → FeatureDocument → Ribbon/Page
- 前一期「草图标配 + 扫描硬化」已交付；TODO 中列出可硬化债与备选包 A（用户基准面）
- 宿主 ABI 当前 `0x00012700`（1.39.0）

## 原始需求

按推荐开干「硬化 + 基准面入门」：

1. Convert 真面边界（ShapeQuery）
2. 真实 UpToVertex 拾取
3. Draft 可选中性面
4. Offset 孔环 + 自交拒绝
5. 多边形边数 3–24
6. 用户基准面（等距面 + 三点）；特征树 + 双击开草图

## 边界确认

| 做 | 不做 |
|----|------|
| 上表 1–6 | 引导线扫描、Rib、特征级阵列、完整坐标系/轴 |
| Part 内参考几何试点 | 装配配合、工程图尺寸 |
| 必要时 bump Host ABI | 大改 PM 面板 / Suppress |

## 需求理解

- Convert 今日为「体全部边 + 共面过滤」，应改为「所选 face 的边界边」
- UpToVertex 今日取边折线首点，应改为靠近点击的边端点（MVP，不强制完整 Vertex 拓扑索引）
- Draft 中性面今日写死世界 Z，应侧栏点选平面面
- Offset 仅最大外环且无自交检测
- 多边形固定 6 边
- 无用户 DatumPlane；仅有虚拟原点三平面

## 疑问澄清（已按推荐自决）

| 问题 | 决策 |
|------|------|
| Vertex 是否完整 TopExp 索引？ | MVP：边拾取 + 近端点；Ref 带回 hit 点 |
| 基准面创建 | 等距面 + 三点；不做成角 |
| 基准面是否进 Parametric rebuild tip？ | 否：FeatureDocument 参考节点，仅提供草图平面 |
| Offset 孔环 UX | 对外环与孔环均偏移（同距；孔环方向取与外环相反号使孔缩小/扩大语义一致） |
