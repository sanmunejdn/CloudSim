# FINAL — 网页端 React 轨迹对齐

## 交付摘要

React 轨迹壳补齐与 fallback 的关键行为差：转换工件可选插入坐标系、拾取高亮可清、应用/生成后生成与编辑页初始化。常读开发约定写入 `web/cloudsim-web-ui/DEVELOPER_GUIDE.md`。

## 关键改动

| 区域 | 路径 |
|------|------|
| 帧列表 | `src/docks/robot/sceneFrames.ts`、`OpParamForm.tsx` |
| 高亮 | `src/scene/SceneViewport.tsx` |
| 会话复位 | `src/state/trajectoryStore.tsx`、`dockNavStore.tsx` |
| 面板 | `TrajectoryGenPanel.tsx`、`TrajectoryEditPanel.tsx` |
| 文档 | `docs/网页端React轨迹对齐/*`、`DEVELOPER_GUIDE.md`、`docs/README.md`、`MODULE_DEVELOPER_GUIDES.md`、`DIRECTORY_LAYOUT.md` |

## 构建

- 前端：`npm run build:debug` / `build:release` → `bin\x64d\web` / `bin\x64\web`
- 本轮无 C++ 变更；若连带编 Host，仍须 Debug\|x64 + Release\|x64
