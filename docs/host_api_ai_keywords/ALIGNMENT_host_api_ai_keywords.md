# ALIGNMENT — Host 公共接口接入 AI（按钮名 = keywords）

## 原始需求

将 Host（Plugin Host）公共接口交给 AI 助手；关键词为对应 Dock/菜单调用按钮的中英文名称。范围：点云 + 几何 + 特征构建 + 标注；完整竖切（Catalog + rules + Executor + domain + 文档）。

## 项目理解

- AI ABI：`CloudSimAiSDK`；运行时：`CloudSimPluginHost` 编入 `CloudSimHost.dll`
- 现有 Catalog 仅 6 个 API；`pointcloud.ops` / `document.import` 域 id 已预留未注册
- Dock 按钮经 `IPluginPointCloudHost` / `IPluginGeometryHost` / `IPluginLabelingHost` / `IPluginHostContext` 调用

## 边界

| 纳入 | 排除 |
|------|------|
| 有调用按钮的 Host API | 无按钮的 Host 方法 |
| keywords = 按钮 EN/ZH | 纯 UI（刷新列表等） |
| rules 最长匹配 + ActionPlan 执行 | 本轮不做专模训练 |

## 已确认决策

- 域：`pointcloud.ops`、`document.import`、`geometry.ops`、`feature.build`、`labeling.annot`
- 选中对象：`args.backend_id` 优先，否则主窗口树选中
- 异步 API：UI 线程 `QEventLoop` 等待回调
