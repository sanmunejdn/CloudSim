# CONSENSUS — 场景层级块式显示

## 已确认决策

| 项 | 结论 |
|----|------|
| 目标页 | Workspace Dock「场景层级 / Scene graph」 |
| 块形式 | **B**：对齐自定义设备组装画布 Link 块视觉 |
| 布局 | **3**：画布 + 父子连线（非表格树、非缩略图网格） |
| 编辑能力 | **只读**：不可增删块、不可接线；支持平移/缩放/点选高亮/可选拖块浏览 |
| 块文案 | 标题=`name`；副标题=`className`；选中时底栏或 tooltip 显示 `localMatrixSummary` |
| 数据源 | 不变：`page->render().sceneGraphSnapshot(256)` |
| 范围外 | Units 树、网页端、`OsgSceneHierarchy` 运行时逻辑、3D 联动选中 |

## 验收标准

1. 「场景层级」Tab 不再出现双列表格；呈现圆角块 + 贝塞尔父子连线。
2. 视觉与 `CustomDeviceAssemblyCanvasWidget` 接近（尺寸约 160×72、圆角、左右端口、背景 `#EEF1F5`）。
3. 有文档时自动按树做左→右分层布局；无文档/空场景显示「无场景」。
4. 滚轮缩放、中键或 Alt+左键平移；切回 Tab / 刷新仍只在可见时取快照。
5. `Widget/DEVELOPER_GUIDE.md` 已更新；Debug\|x64 与 Release\|x64 编译通过。

## 技术约束

- 新建 `SceneHierarchyCanvasWidget`（Widget 模块），**不**复用组装画布的编辑/关节数据 API。
- 连线不使用旋转/移动副标签，避免语义混淆；线色对齐组装画布默认蓝。
- 节点过多时截断显示（建议上限 200），底栏提示已截断。
- 不抽公共基类（单消费者，避免过度抽象）。
