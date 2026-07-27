# ALIGNMENT — 工艺流程仿真插件（MVP）

## 原始需求

开发工艺流程仿真插件：菜单进入后中央用自研节点画布替换 3D 显示；右侧仅节点库；隐藏原右侧 Workspace/AI/插件页。不接 3D，不做真实仿真引擎。画布参考 NodeFlowDemo 逻辑，不引入其头文件/动态库。

## 项目理解

- CloudSim：Qt 5.14 + 插件体系；插件仅链 `CloudSimPluginSDK`
- 中央 3D 在 `DocumentHost` 的 `OsgWidget`；右侧为 `m_unitDock`（Workspace/AI/插件 Tab）
- 现有插件 API 无中央视口切换；需宿主 ABI 扩展

## 边界确认

| 纳入 MVP | 排除 |
|----------|------|
| 自研 `ProcessFlowCanvasWidget` | 引入 NodeFlowWidget 头/库 |
| 菜单进入/返回 + 中央堆栈切换 | 永久替换 3D |
| 进入时右侧仅节点库 | 节点绑机器人/程序 |
| 拖拽/连线/缩放/删/导出 JSON | 写入 `.pcp`、仿真引擎 |

## 关键假设

1. 每文档独立画布；返回 3D 不销毁图数据
2. 宿主版本升至 1.17.0（vtable 末尾追加）
3. 左侧属性 Dock 进入工艺流程后保持可见
