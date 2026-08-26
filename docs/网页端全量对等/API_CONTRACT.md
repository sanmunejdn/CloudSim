# 网页端全量对等 — API 契约

> W0 协调交付。所有新路由在 `WebGatewayParity.cpp` 注册；点云在 `WebGatewayPointCloud.cpp`；轨迹在 `WebGatewayTrajectory.cpp`。

## 通用约定

| 项 | 说明 |
|----|------|
| 编码 | UTF-8 JSON |
| 成功 | `"ok": true` |
| 失败 | `"ok": false`，`"error": "..."`；HTTP 400（部分路由） |
| GUI 线程 | 写 Host/Document 的路由经 `QMetaObject::invokeMethod(..., BlockingQueuedConnection)` |
| SSE | `GET /api/events`，`data: {json}\n\n`；无客户端时不入队（背压） |

### SSE 事件

| type | 载荷 |
|------|------|
| `ready` | `role: web` |
| `SceneChanged` | — |
| `ProjectLoaded` | `path`, `objectCount` |
| `SelectionChanged` | `backendId` |
| `PlaybackFrame` | `running`, `joints`, `instructionId`… |
| `GeometryJobProgress` | `percent`, `message` |
| `WorkspaceModeChanged` | `mode` |
| `EventsDropped` | 队列溢出通知 |

---

## W1 — 机器人回放 / 导出 / 几何 / AI

### `POST /api/robot/run`

```json
{ "sceneRootBackendId": "uuid", "programId": "optional", "playbackRate": 1.0 }
```

响应：`{ "ok": true, "status": "running" }`

### `POST /api/robot/stop`

响应：`{ "ok": true }`

### `GET /api/robot/playback/status`

响应：`{ "ok": true, "running": bool, ... }`

### `POST /api/robot/export`

```json
{ "sceneRootBackendId": "uuid", "brand": "ABB|KUKA|...", "outputPath": "optional" }
```

### `POST /api/geometry/op`

分发字段 `op`：`discretize` | `boolean` | `intersect` | `mesh-from-curve`

### `POST /api/geometry/discretize`

```json
{ "stepPath": "...", "backendId": "...", "meshParams": { } }
```

### `POST /api/ai/chat`

```json
{ "message": "...", "domain": "scene.ops|robot.command|process.flow", "systemPrompt": "optional" }
```

响应：`{ "ok": true, "assistantText": "..." }` 或配置缺失错误。

### `GET /api/ai/status`

桌面 LLM 配置探测（非 stub 假回复）。

---

## W2 — 碰撞 / 程序编辑 / 设备

### `GET|PUT /api/robot/collision-settings`

读写 `RobotCollision::Settings` JSON。

### `POST /api/robot/collision/plan` | `/confirm`

规划预览与确认插入程序。

### `POST /api/robot/programs/switch`

```json
{ "sceneRootBackendId": "uuid", "programId": "..." }
```

### `POST /api/robot/program-edit/undo` | `/redo`

```json
{ "sceneRootBackendId": "uuid" }
```

### `GET|PUT /api/devices/plc` | `/api/devices/camera`

设备列表侧车（V1 可返回空数组 + 配置 JSON）。

---

## W3 — 工作区侧车

### `GET|PUT /api/processflow/graph`

`processFlow` 侧车图 JSON。

### `POST /api/processflow/sim/run`

DES 仿真启动。

### `POST /api/drawing/export`

```json
{ "format": "svg|dxf|pdf", "outputPath": "optional" }
```

### `GET /api/geomodeling/summary`

特征树 / Body 摘要（GM-1）。

### `GET /api/labeling/tasks`

标注任务列表。

### `GET|PUT /api/sidecar/<key>`

`processFlow` | `geometricModeling` | `annotations` | `workspaceMode`

---

## W4 — 装配 / 帮助 / i18n

### `POST /api/assembly/mate`

```json
{ "grounded": { "brepId", "faceIndex" }, "moving": { ... }, "params": { } }
```

V1：占位或调用 `AssemblyMateApply`（需 Host 链几何）。

### `GET /api/help`

帮助索引 JSON。

### `GET /api/i18n/zh` | `/api/i18n/en`

`{ "lang", "strings": { "key": "value" } }`

---

## 分支与文件分区

见 [AGENT_BRANCHES.md](./AGENT_BRANCHES.md)。各 Agent 仅改对应 `WebGateway*.cpp` 与 Headless Bridge，禁止多人改同一未拆分文件。
