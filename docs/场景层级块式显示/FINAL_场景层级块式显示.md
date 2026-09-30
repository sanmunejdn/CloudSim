# FINAL — 场景层级块式显示

## 摘要

将 Workspace「场景层级」从双列 `QTreeWidget` 改为与自定义设备组装画布同风格的只读块式画布：圆角 Link 块 + 父子贝塞尔连线，自动左→右分层布局。

## 主要变更

| 文件 | 变更 |
|------|------|
| `SceneHierarchyCanvasWidget.h/.cpp` | 新建只读画布 |
| `MainWindowUiSetup.cpp` | Tab 挂载画布 |
| `MainWindowBackendTree.cpp` | `refreshOsgSceneTree` → `setSceneGraph` |
| `MainWindow.h` / `MainWindow.cpp` | 成员类型与语言切换 |
| `Widget.vcxproj` (+filters) | 纳入新源文件 |
| `Widget/DEVELOPER_GUIDE.md` | §5.3 说明更新 |

## 编译

| 配置 | 结果 |
|------|------|
| Debug\|x64 | ✅ `bin\x64d\Widget.dll` |
| Release\|x64 | ✅ `bin\x64\Widget.dll` |

## 设计取舍

- 不复用 `CustomDeviceAssemblyCanvasWidget`（其绑定关节编辑 API）
- 连线无旋转/移动副标签，避免调试树语义混淆
- 节点上限 200，超出底栏提示截断

## 后续完整落地（命名 + 双击属性）

| 项 | 说明 |
|----|------|
| 契约 | `IRenderView::SceneNodeInfo` 增 `displayName`/`backendId`/世界矩阵/渲染摘要等 |
| 快照 | `OsgRenderViewAdapter` 读 `BackendIdUserData` + `listObjectSnapshots` |
| UI | `OsgSceneNodeI18n`；双击详情对话框；`backendNodeActivated` → 选中联动 |
| 编译 | Debug/Release Widget 已通过 |
