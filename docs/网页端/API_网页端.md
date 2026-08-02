# CloudSim Web Gateway API 草表

基址默认 `http://127.0.0.1:8787`。事件为 SSE：`GET /api/events`（`text/event-stream`，`data: {json}`）。

## 公共

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/api/health` | `{ok,role:"web",pid,port}` |
| GET | `/api/events` | SSE |

## P1 工程 / CAD

| 方法 | 路径 | Body / 说明 |
|------|------|-------------|
| POST | `/api/project/new` | 清空文档 |
| POST | `/api/project/open` | `{path}` |
| POST | `/api/project/save` | `{path?}` 复用 `buildProjectSaveRoot` + STORE zip |
| GET | `/api/objects` | 对象树快照 + pose |
| GET | `/api/objects/:id` | 详情 + properties |
| PATCH | `/api/objects/:id` | `{visible?,pose?,properties?,propertyKey,propertyValue}` |
| DELETE | `/api/objects/:id` | 删子树 |
| POST | `/api/objects/import` | `{path,isPointCloud?}` |
| POST | `/api/objects/register` | `{className,name,parentId?}` |
| POST | `/api/objects/attach` | `{parentId,childId}` |
| GET | `/api/mesh/:id` | triangle soup `float32` xyz |
| POST | `/api/selection` | `{backendId}` |

SSE：`ProjectLoaded` `ProjectSaved` `PoseCommitted` `ObjectPatched` `BackendObjectRegistered` `BackendObjectRemoved` `SelectionChanged`

## P2 机器人

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/api/robot/urdf/import` | `{urdfPath}` → `registerUrdfRobot` |
| POST | `/api/robot/joints` | `{sceneRootBackendId,jointAnglesRad[]}` |
| GET/PUT | `/api/robot/programs` | 程序 JSON |
| POST | `/api/robot/plan` | 指令规划 |
| POST | `/api/robot/run` `/api/robot/stop` | 回放控制（客户端编排） |
| POST | `/api/robot/export` | Canonical 导出管道占位 |

SSE：`RobotKinematicsApplied`

## P3 几何 / 点云

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/api/geometry/op` | `{op,targetId,…}` 作业面 |
| POST | `/api/pointcloud/op` | 同上 |
| GET | `/api/pointcloud/chunk/:id` | LOD 二进制块（禁止整云 JSON） |

SSE：`GeometryJobProgress` `PointCloudJobProgress`

## P4 模式 / 侧车

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/api/modes` | 模式目录 + active |
| GET/PUT | `/api/sidecar/processFlow` | 工艺侧车 |
| GET/PUT | `/api/sidecar/geometricModeling` | 建模侧车 |
| GET/PUT | `/api/sidecar/annotations` | 标注数组 |
| GET/PUT | `/api/sidecar/workspaceMode` | `{mode}` |

SSE：`WorkspaceModeChanged`

## P5 AI / 杂项

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/api/ai/status` | |
| POST | `/api/ai/chat` | LLM 桥接（配置对齐桌面 `ai_config.json`） |
| GET | `/api/help` | 帮助索引 |
| GET | `/api/i18n/:lang` | 前端字符串包 |
| GET | `/api/devices/plc` `/api/devices/camera` | 设备面板占位 |
