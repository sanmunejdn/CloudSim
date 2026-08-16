# CloudSim Web UI（Vite + React）

正式静态根：`CloudSimWeb.exe` 旁的 `web/`（仓库 `bin\x64d\web` / `bin\x64\web`）。  
行为以 `_archive/public-fallback/` 为金标对照；默认部署产物来自本目录源码，**不要**只改 fallback 或只拷到 `CloudSim\bin\...\web`。

专题进度见 [`docs/网页端信号网络与自定义设备/`](../../docs/网页端信号网络与自定义设备/)、[`docs/网页端设备页桌面同步/`](../../docs/网页端设备页桌面同步/)；旧单表 IO 对等见 [`docs/_archive/网页端IO信号对等/`](../../docs/_archive/网页端IO信号对等/)。

## IO / 自定义设备（网页）

- 工程侧车：`ioSignalNetwork`（多 Owner + wires）；旧 `ioSignals` 打开时迁入主机器人 Owner
- API：`/api/io/network*`、`/api/custom-devices*`；旧 `/api/io/signals*` 薄封装到主机器人
- **右栏「设备」**：顶栏「机器人 | 自定义设备」；机器人侧指令/轴/轨迹/坐标系；自定义设备侧「设备指令」（姿态库 + DI→姿态绑定）与「轴控制」
- **左栏「设备」**：URDF 目录 + 组装入口（运行面在右栏）
- 程序 Run：客户端编排对 `SET_DO` / `SET_AO` / `WAIT(IO)` 写同一网络 runtime
- 导航：`dockNavStore`（`ws=devices`、`deviceMode`、`deviceTab`）；选中设备：`deviceRuntimeStore`

## 自定义设备组装（对齐桌面）

对照：[`docs/网页端设备页桌面同步/ASSEMBLY_桌面对照.md`](../../docs/网页端设备页桌面同步/ASSEMBLY_桌面对照.md)。

- UI：`CustomDeviceAssemblyDialog` — 从场景 / 导入模型、设固定、关节属性（移动/旋转、限位、轴、旋转中心）、Apply
- API：`ensure` / `attach` / `assembly-candidates` / `POST /api/custom-devices`（提交前 Host 挂父子再 `commitGraph`）/ `export-urdf`
- 左栏「新建/组装」传空 id；「编辑组装」传选中设备 id

## 打开模型

- 菜单 **文件 → 打开模型…**（视口工具条「模型」同入口）：`dialogOpen({ purpose: "model" })` → 多选 `paths[]` → 逐个 `/api/objects/import` 且 **`isPointCloud: false`**
- 过滤器对齐桌面：obj/stl/ply/off/dxf/dae/3ds/fbx/step/stp/igs/iges
- 组装对话框「导入模型…」同样多选
- 与「导入…」区分：后者也可多选，并按扩展名走点云；打开模型始终走网格/CAD 路径

## 日常开发

1. 启动 Host：`bin\x64d\CloudSimWeb.exe`（或 Release）监听 `8787`
2. 前端热更：`cd CloudSim/web/cloudsim-web-ui && npm run dev`（5173，代理 `/api` → 8787）
3. 或只编 Host：PostBuild 会跑 Vite，产物进 `$(CloudSimBinDir)web`（`bin\x64d\web` / `bin\x64\web`）
4. 手工验证前端时 **Debug 与 Release 都要构建**：

```bash
npm run build:debug    # → 仓库根 bin/x64d/web
npm run build:release  # → 仓库根 bin/x64/web
```

## PostBuild 环境变量

| 变量 | 作用 |
|---|---|
| `CLOUDSIM_WEB_SKIP_BUILD=1` | 跳过前端构建（纯 C++ 迭代） |
| `CLOUDSIM_WEB_FALLBACK=1` | 临时 xcopy `_archive/public-fallback`（紧急回退） |

默认：Debug → `npm run build:debug`；Release → `npm run build:release`。失败则 MSBuild 失败。

若系统无 Node，可将便携 Node 解压到 `web/cloudsim-web-ui/.tools/node`（含 `node.exe` / `npm.cmd`），PostBuild 会自动使用。

## 目录约定

```text
src/
  api/          # REST 封装（objects / robot / trajectory …）
  docks/robot/  # 机器人：指令树、轴、轨迹、坐标系
  docks/devices/# 左栏目录/组装；右栏 DeviceCommandPanel
  docks/signals/# Owner 信号表 + 连接站
  scene/        # Three 视口、mesh、帧轴、Raw 预览、拾取高亮
  state/        # scene / trajectory / robotProgram / dockNav / deviceRuntime …
  shell/        # 左右坞壳
  styles/       # shell.css（对齐 fallback）
_archive/public-fallback/   # 旧单文件壳，对照用，不参与默认部署
```

构建：`vite.config.ts` 按 mode 输出到 `bin\x64d\web` 或 `bin\x64\web`。

## 场景与空间

- 后端姿态用 **`worldMatrix`**（勿只用 pose 字段拼连杆）；材质用 Phong + `color`。
- 场景组应用 `zUpToYUp`；坐标系对象 `FrameBackendData` 由客户端画 RGB 短轴（`frameAxes.ts`）。
- TCP / 用户系叠加：`GET /api/robot/frames/overlays`。
- URDF 导入体字段：`urdfPath`（不是 `path`）。

## 轨迹生成（`TrajectoryGenPanel`）

| 能力 | 说明 |
|------|------|
| PathPlan | `create` / `bind` / `begin-edit` / `cancel-edit` |
| 拾取 | `face` / `edge` → `/api/pick/hover`；策略按 Face/Line 过滤 |
| 特征参数 | `GET /api/trajectory/feature-schema` → `FeatureParamForm` |
| 离散 | 防抖自动离散；结果经 `publishRawPreview` 画点/线/姿态轴 |
| 编辑门闩 | `featureEditActive===false` 时锁定拾取与参数 |

**开始修改**：从会话 `sourceFeatureJson` 灌入特征表（对齐桌面）。

## 轨迹编辑（`TrajectoryEditPanel`）

| 能力 | 说明 |
|------|------|
| 算子调色板 | `/api/trajectory/op-palette`（字段 `ops`）；空则用 `OP_PALETTE_FALLBACK` |
| 参数表单 | `/api/trajectory/op-schema` → `OpParamForm` |
| 作用域 | 对齐桌面：`CommonScope` 全程序/分组/P 范围；`visibleWhenScopeKind` 用令牌匹配（见 `scopeKindToken`）；有 Raw 时新建算子默认 `PointIndexRange` `1…N` |
| ToWorkpieceInHand | 「外部 TCP」用 `sceneFrames.fetchSceneCoordinateFrames`：Host 登记帧 → 本地对象 → `/api/objects`；UI 为 radio 列表（非原生 select） |
| 预览/应用/撤销 | `preview` / `apply` / `reset` / `undo` / `redo` |

离散后改区间点：作用域选「P 范围」→ 调 P 起/止 → 启用预览。Raw 阶段选「分组」会提示改用 P 范围。

### 应用 / 生成后必须初始化（对齐 `exitTrajEditUiAfterCommit`）

成功 `emit` / `apply` 后调用 `trajectoryStore.exitEditAfterCommit()`：

1. 清空特征表、拾取模式、`featureEditActive`
2. `publishRawPreview(null)` + 清除拾取高亮
3. 递增 `editUiEpoch`（生成/编辑面板复位本地 UI）
4. 应用额外清空流水线本地态；切回指令树（`goCmd`）

绑定 PathPlan / 取消修改同样走清空路径，勿只 `syncSession` 而残留 React 本地态。

## 拾取高亮

- 事件：`cloudsim-pick-highlight`（`detail.clear` 或空内容即清除）
- 悬停预览仅在 `pickMode` 有效时绘制；未命中 / 指针离开 / 提交拾取后清除
- 用 `hoverPickSeqRef` 丢弃过期异步 hover，避免退出拾取后又画回蓝面

## 3D 轨迹显示

| 阶段 | 显示 |
|------|------|
| 离散后（编辑中） | `publishRawPreview` → 青色折线 + 点/姿态轴（`rawPreview` 组） |
| 应用/生成后 | 清 Raw；`instrMarkers` 画程序内 PTP/LINE/ARC 路点（绿点+RGB 轴+折线） |
| Raw 预览开启时 | 隐藏指令路点，避免双轨 |

实现：`scene/instrMarkers.ts`、`SceneViewport` 监听程序目录与 `cloudsim-raw-preview`。

## 指令属性面板

- 选中指令后左坞切到「属性」；标题 `指令属性 · <id>`
- 字段与中文标签对齐桌面 `InstructionPropertyPanel` / `propertyDisplayLabelForKey`（`docks/props/propLabels.ts` + `instrPropView.ts`）
- 过滤 `context.*` / `render.*`；轴配置枚举用下拉；数值三位小数
- Host：`HeadlessInstructionPropertyDelegate` 递归查找嵌套 if/while 内指令

## 关键状态

| Store | 职责 |
|-------|------|
| `trajectoryStore` | PathPlan、特征、编辑门闩、`exitEditAfterCommit` / `editUiEpoch` |
| `robotProgramStore` | 程序目录、指令树 |
| `dockNavStore` | 右坞 Tab；`goTrajGen` / `goCmd` |
| `sceneStore` | 对象列表、选择、交互模式 |

## 要求

- Node 18+（或 `.tools/node`）
- 首次 PostBuild 会 `npm ci` / `npm install`
- 行为歧义时以 `_archive/public-fallback/app.js` 为准，再改 React
