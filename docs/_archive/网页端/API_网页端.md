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
| POST | `/api/robot/urdf/import` | `{urdfPath}` → `registerUrdfRobot`（需 HeadlessRobotContext） |
| GET | `/api/robot/instances` | `[{sceneRootBackendId,label,jointCount,urdfPath}]` |
| GET | `/api/robot/joints?sceneRootBackendId=` | `joints[]`：`{name,lowerRad,upperRad,angleRad}`（缺省取首实例） |
| GET | `/api/robot/resolve?backendId=` | `{isRobot,sceneRootBackendId,anchorBackendId,flangeBackendId}` |
| POST | `/api/robot/place` | `{anchorBackendId,worldMatrix[16]}` 反解基座并 FK 全连杆（gizmo 拖动） |
| POST | `/api/robot/tcp-ik` | `{flangeBackendId,worldMatrix[16]}` 末端 IK → 关节 |
| GET | `/api/robot/tcp-pose?sceneRootBackendId=` | 当前 TCP：`positionMm[]`/`eulerDeg[]`/`jointRadCsv` |
| POST | `/api/robot/joints` | `{sceneRootBackendId,jointAnglesRad[]}` → FK + SSE |
| GET/PUT | `/api/robot/programs` | 程序 JSON（`[{sceneBackendId,activeProgramId,programs:[{id,instructions[]}]}]`） |
| GET | `/api/robot/instructions/:id/properties` | 指令属性行 |
| PATCH | `/api/robot/instructions/:id` | `{key,value}` 改属性 |
| POST | `/api/robot/plan` | `{instructionType,targetPose,jointRadCsv,sceneRootBackendId,extensions?}`；种子取 `jointRadCsv` |
| POST | `/api/robot/run` `/api/robot/stop` | 回放控制（stub，后续 Executor） |
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
| GET | `/api/devices/catalog` | 扫描 `applicationDir/resource/models`：`types[]`、`brandsByType`、`packages[]`（`type,brand,name,urdfPath,thumbnailUrl?`） |
| GET | `/api/devices/thumbnail?path=` | 本地缩略图（须在 models 根下） |
| GET | `/api/devices/plc` `/api/devices/camera` | PLC/工业相机面板占位（非 URDF 设备库） |

## P6 轨迹 / PathPlan / 拾取

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/api/trajectory/session` | phase / raw 摘要 / pipelineOpCount / canUndo·Redo / `sourceFeatureJson` |
| GET | `/api/trajectory/path-plans?sceneRootBackendId=` | PathPlan 列表 |
| POST | `/api/robot/path-plan` | `{sceneRootBackendId}` 新建并绑定 |
| POST | `/api/trajectory/bind` | `{pathPlanId}` |
| POST | `/api/trajectory/begin-edit` `/cancel-edit` | 「开始修改」门闩 |
| POST | `/api/pick/mesh-element` | BREP 射线拾取：`{mode:edge\|face,workpieceBackendId,originMm[],dir[]}` |
| POST | `/api/pick/hover` | 同上，额外返回 `polylinesWorld`（世界系折线，供高亮；不写特征表） |
| GET | `/api/trajectory/op-schema?kind=&opIndex=` | 算子参数 schema（`fields[]`）+ 当前 `values`（对接 TrajectoryOpParamSchema） |
| GET | `/api/trajectory/feature-schema?strategyId=` | 离散策略 schema：`fields`+`defaults`+中文名；无 strategyId 时返回策略目录 |
| GET | `/api/trajectory/feature-catalog?workpiece=` | `enumerateFeatureCatalog` JSON |
| POST | `/api/trajectory/discretize` | FeatureList v2 → Raw |
| POST | `/api/trajectory/mesh-spec` | MeshTrajectorySpec → Raw |
| GET/PUT | `/api/trajectory/pipeline` | 算子描述符数组 |
| POST | `/api/trajectory/recipe` | `{recipe:weld\|glue\|grind}` |
| POST | `/api/trajectory/preview` | Raw 世界折线：`pointsMm` + 每点 `eulersDeg`（前端按轴间隔稀疏画 TCP 轴） |
| POST | `/api/trajectory/apply` | emit LINE + PathPlan Applied |
| POST | `/api/trajectory/reset` | 清空管线草稿 |
| POST | `/api/trajectory/undo` `/redo` | 会话草稿栈 |
| GET | `/api/trajectory/templates/{kind}` | 列出（`pipeline` / `discretize`） |
| GET/POST/DELETE | `/api/trajectory/templates/{kind}/{name}` | 读写删（AppData `CloudSim/templates`） |

## 原生对话框

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/api/dialog/open` | `{kind:project\|folder\|save\|import\|urdf}` → 本机绝对路径 |
