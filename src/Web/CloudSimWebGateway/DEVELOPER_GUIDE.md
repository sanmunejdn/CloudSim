# CloudSimWebGateway 模块开发文档

静态库（链入 `CloudSimWeb.exe`）：HTTP/REST + SSE，托管 `{exe}/web`，业务经 Headless `DocumentHost` / Data / 轨迹会话，与桌面同源。

API 面归档：[`docs/_archive/网页端/API_网页端.md`](../../../docs/_archive/网页端/API_网页端.md)。前端：[`web/cloudsim-web-ui/DEVELOPER_GUIDE.md`](../../../web/cloudsim-web-ui/DEVELOPER_GUIDE.md)。

## 1. 定位与边界

| 项 | 约定 |
|----|------|
| 进程 | `CloudSimWeb.exe` 独立，不监听桌面端口 |
| 默认 | `http://127.0.0.1:8787`（`--port=` 可改） |
| 静态根 | `m_config.staticRoot` → 通常 `{OutDir}web` |
| 线程 | httplib 在 `serverThread`；业务经 `QMetaObject::invokeMethod` 回 GUI 线程 |
| 停服 | `stop()`：`running=false` → `svr.stop()` → `wakeAll` → **join** `serverThread` |

勿在 Gateway 内堆积桌面 Widget / OSG 类型；Headless 适配在 Host。

## 2. SSE 与事件队列（已知风险）

实现：`WebGateway.cpp` 中 `eventQueue` + `eventMutex` + `eventCv`；`GET /api/events` 消费 `takeFirst`。

| 现状 | 说明 |
|------|------|
| `pushEvent` | 队列上限 256；满则丢最旧，约 1s 节流插入 `EventsDropped` |
| `sseClients` | 仅计数，**不**门控入队 |
| `start()` | 对 `DocumentHost::visualSceneDirty` `connect` → `SceneChanged`；`stop()` **对称 disconnect** |

高负载（无客户端 / 慢客户端 / 反复 start）下队列与重复 handler 可能膨胀。行为修复清单见仓库审查记录；落地前勿假定队列有界。

## 3. 路由分层（实现内分区）

| 区 | 内容（摘要） |
|----|----------------|
| 工程 / 健康 | open/save/health、工程锁 |
| 机器人 | joints、TCP IK、坐标系、程序 |
| 几何 / 点云 | `/api/geometry/op`、`/api/pointcloud/op`（作业面；重活在 Host） |
| 模式 / 侧车 | workspace、io network、custom devices |
| AI / 设备杂项 | 助手、帮助、设备指令对齐面 |

## 4. 与 Host / 前端

- 真源：`DocumentHost`（Headless）+ `HeadlessTrajectorySession` 等
- 前端经 REST + SSE；勿在 React 再维护第二套后端对象图
- 构建：`CloudSimHost` → 本库 → `CloudSimWeb`；**Debug\|x64** 与 **Release\|x64** 都要编

## 5. 注释约定

分区标签用中文短句（如 `// 机器人 API`）；禁止复述代码的旁白。全局约定见 [`docs/MODULE_DEVELOPER_GUIDES.md`](../../../docs/MODULE_DEVELOPER_GUIDES.md)「源码注释约定」。
