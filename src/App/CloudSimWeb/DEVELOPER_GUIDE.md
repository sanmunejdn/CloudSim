# CloudSimWeb 开发指南

## 1. 定位

| 项 | 说明 |
|----|------|
| 解决方案 | `CloudSimWeb.sln` |
| 产物 | `CloudSimWeb.exe` |
| 职责 | Qt 事件循环；创建 Headless 组合根；启停 `CloudSimWebGateway`；托管 `bin/*/web` |

## 2. 边界

- **依赖**：CloudSimHostHeadless、CloudSimWebGateway、Data / Robot / Geometry 等共享 DLL
- **不负责**：React 前端源码（见 `web/cloudsim-web-ui`）；桌面 Widget UI

## 3. 已交付能力

- 默认监听 `http://127.0.0.1:8787`（`--port=` 可改）
- 静态托管 `{exe}/web`；REST / SSE 经 Gateway
- 与桌面同源 Data / 轨迹 / 几何建模 API（Headless 桥）

## 4. 验证

Debug|x64 与 Release|x64 均编 `CloudSimWeb.vcxproj`；前端 `npm run build:debug` / `build:release`。

## 5. 相关文档

- [CloudSimWebGateway](../../Web/CloudSimWebGateway/DEVELOPER_GUIDE.md)
- [cloudsim-web-ui](../../../web/cloudsim-web-ui/DEVELOPER_GUIDE.md)
- [网页端专题](../../features/网页端/README.md)
- [solutions/CloudSimWeb.sln.md](../../solutions/CloudSimWeb.sln.md)
