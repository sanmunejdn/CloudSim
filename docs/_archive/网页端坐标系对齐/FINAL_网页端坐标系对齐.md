# FINAL — 网页端坐标系对齐

## 交付摘要

网页「坐标系」Tab 对齐桌面 `RobotFrameSettingsWidget` 核心能力：工具/用户系 CRUD、法兰 link、位姿编辑、设为当前、TCP 捕获、重置、三维显示开关、Three.js 叠加轴；Gateway REST + SSE；工程保存回写 kinematics。

## 关键改动

| 区域 | 路径 |
|------|------|
| Host | `RobotCoordinateFrameOps.{h,cpp}`、`ProjectPackageIo.h`（声明 merge kinematics） |
| Gateway | `WebGateway.cpp` / `WebGatewayApi.cpp` / `WebGateway.h` |
| 链接 | `CloudSimWeb.vcxproj` 增加 `RobotScene.lib`（Gateway 静态库引用 `RobotCoordinate` 符号） |
| Frontend | `public-fallback/{index.html,app.js,styles.css}` |
| 文档 | `docs/网页端坐标系对齐/*`、`docs/_archive/网页端/API_网页端.md`、`TODO_网页端功能对等.md` |

## 构建

- Debug：`bin\x64d\CloudSimWeb.exe` + `bin\x64d\web\`
- Release：`bin\x64\CloudSimWeb.exe` + `bin\x64\web\`
