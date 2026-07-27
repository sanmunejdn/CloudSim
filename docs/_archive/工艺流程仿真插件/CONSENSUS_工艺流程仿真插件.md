# CONSENSUS — 工艺流程仿真插件（MVP）

## 需求与验收

1. 菜单「工艺流程」→「进入工艺流程」/「返回三维场景」
2. 进入：中央为自研画布；右侧仅节点库；`m_unitDock` 隐藏
3. 返回：中央 3D；右侧原面板恢复；节点库隐藏；图数据保留
4. 可添加/拖拽/连线/缩放/删除/导出 JSON；无 NodeFlowDemo 链接依赖；不碰 OSG

## 技术方案

- `DocumentHost`：`QStackedWidget`（OsgWidget / alternate）
- `MainWindow`：`enterProcessFlowSideUi` / `exitProcessFlowSideUi`
- `IPluginHostContext` 1.17.0 追加中央与右侧 API
- `ProcessFlowPlugin`：自研画布 + 节点库，仅链 PluginSDK

## 任务边界

不实现仿真引擎、机器人绑定、工程内嵌持久化。
