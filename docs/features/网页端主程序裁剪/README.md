# 网页端主程序裁剪

网页端暂时只保留**主程序**功能（核心 + 侧栏工具），不提供几何建模 / 工艺流程 / 标注工作区前端壳与对应 Gateway 专用路由。工程图已从产品移除。

## 行为约定

1. 设置菜单无「工作区模式」切换；UI 始终主程序三维壳。
2. `/api/modes` 仅 `scene3d`；`PUT workspaceMode` 非主程序时归一为 `scene3d`。
3. `/api/geomodeling/*`、`/api/processflow/*`、`/api/drawing/*`、`/api/labeling/*` 返回 404。
4. 右坞点云 / 几何 / 机器人 / AI（scene/robot）仍可用；PLC/相机 stub 保留。
5. `/api/sidecar/processFlow|geometricModeling|annotations` 仍可读写（工程保真）。
6. Host `Headless*Bridge` 源文件不删。

## 明确不做

PLC / 相机深度对等、外置控制器 UI、多视口等 P1/P2（网页侧）。
