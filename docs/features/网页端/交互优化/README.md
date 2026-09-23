# 网页端交互优化

落地 M1–M4（工程/交互层），非全量功能对等。

## 变更摘要

### M1 刷新与视口

- `sceneStore`：SceneChanged / 对象增删等全量 `refreshObjects` **80ms 防抖**
- `projectStore`：对象变更只 `setDirty`，**不再 bump**（避免与 SSE 双拉 objects）；去掉对 `message` 的脏刷新
- `scene/meshLoad.ts`：点云装载、`geomSignature`、`drainGroup` / `clearPickOverlayGroup`
- 视口 unmount 统一 `drainGroup` 释放 mesh / overlay
- **`SceneViewport` 拆分**：`scene/viewport/{useViewportCore,useMeshSync,useRobotTeach,useViewportInteraction,useSceneOverlays}`；主机只编排 refs 与 imperative API

### M2 导航与事件

- `dockNavStore`：左坞 Tab 并入；`focusProps()`；`ROBOT_PRIMARY_TABS` + `ROBOT_MORE_TABS`
- `ui/uiEvents.ts`：领域总线（pick / raw / mate / focusProps），替代 `window` CustomEvent
- `RightDock`：指令/轴/轨迹为主路径，坐标系/外轴/碰撞/通讯收入「更多」

### M3 对话框与工具条

- `ui/Dialog.tsx`：`showConfirm` / `showPrompt`（Esc、遮罩关闭）
- 程序/姿态/工程脏文档等已替换 `window.prompt|confirm`
- 视口工具条增加短标签（选择/聚焦/…）

### M4 拆分

- `workspaces/geomodelRibbon.ts`：Ribbon 工具表
- `robot/ioProgramSteps.ts`：程序 Run 的 IO 解析

## 验证

```bash
cd CloudSim/web/cloudsim-web-ui
npm run build:debug
npm run build:release
```

产物：`bin\x64d\web` / `bin\x64\web`。

## 未纳入（产品债）

- PLC/相机 stub、几何选面体验等能力对等项
