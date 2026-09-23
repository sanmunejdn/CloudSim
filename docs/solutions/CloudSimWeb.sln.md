# CloudSimWeb.sln（网页）

入口：`CloudSim/CloudSimWeb.sln` → `CloudSimWeb.exe` + `bin/*/web`。全库入口：[../README.md](../README.md)。

与桌面共享 Contracts / Data / Robot / Geometry / Infra / 多数 SDK；**不含** Widget / RobotWidget / 工作区模式插件工程。Host 使用 `CloudSimHostHeadless`（源码与桌面 Host 共享，见 [网页端/Host同步](../features/网页端/Host同步/README.md)）。

## Web 专有

| 工程 / 前端 | 指南 |
|-------------|------|
| CloudSimWeb | [DEVELOPER_GUIDE](../../src/App/CloudSimWeb/DEVELOPER_GUIDE.md) |
| CloudSimWebGateway | [DEVELOPER_GUIDE](../../src/Web/CloudSimWebGateway/DEVELOPER_GUIDE.md) |
| cloudsim-web-ui（Vite） | [DEVELOPER_GUIDE](../../web/cloudsim-web-ui/DEVELOPER_GUIDE.md) |

## 共享后端（与桌面同指南）

| 域 | 入口 |
|----|------|
| Host（Headless） | [CloudSimHost](../../src/Host/CloudSimHost/DEVELOPER_GUIDE.md) |
| Data / Robot / Geometry / Contracts | 见 [CloudSim.sln.md](CloudSim.sln.md) |

## 专题

- [网页端总览](../features/网页端/README.md)
- [API_CONTRACT](../features/网页端/全量对等/API_CONTRACT.md)
- [交互优化](../features/网页端/交互优化/README.md)
- [Host ↔ Headless](../features/网页端/Host同步/README.md)
- 产品对照表：[CloudSim/README.md](../../README.md)
