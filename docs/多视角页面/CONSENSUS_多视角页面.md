# CONSENSUS — 多视角页面

## 需求与验收

1. Single / 左右 / 上下 / 四宫格切换，共享场景、各格独立自由视角
2. 方向预设：前 / 顶 / 右 / 等轴
3. 「新窗口打开视角」最多 2 个；关文档销毁
4. 拾取 / 工具栏 / 仿真仍走 Primary `OsgWidget`
5. Debug|x64 + Release|x64 编译通过

## 技术方案

- Primary `OsgWidget` + `SatelliteOsgView`（共享场景根、独立 Viewer）
- `MultiViewShell`（QSplitter）经 Desktop flavor 挂入 `DocumentHost`
- 菜单「视图 → 多视角」控制布局与浮动窗

## 约束

- Headless：`createFlavorMultiViewShell` 返回 nullptr
- 文档内最多 4 格；浮动最多 2 窗
