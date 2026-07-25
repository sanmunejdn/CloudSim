# ALIGNMENT — 工程筛选器整理

## 1. 项目上下文

`CloudSim.sln` 共 **39** 个工程。Solution Explorer 筛选器约定为自定义 **`inc` / `src`**（非 VS 默认 Header Files / Source Files）。

| 现状 | 说明 |
|------|------|
| 主流模式 | 几乎全部文件只挂在扁平 `inc`、`src` |
| 已有细分子夹 | `RobotScene`（`resource\...`）、`GeometryAlgorithm`（`discretizers`）、部分插件 `Plugin`/`Global`、`PointCloudAlgorithm`（`sdf`/`spare`） |
| 磁盘已有功能目录但筛选器未镜像 | `TrajectoryAlgorithmBuiltins\ops\<OpName>\`、`GeometryAlgorithm\source\<流水线>\`、`IndustrialCameraSDK\source\{calib,hik,mech,pose}`、`ProcessFlowPlugin\inc\sim` 等 |
| 缺 `.filters` | CollisionAlgorithm、RobotCommSDK、IndustrialCameraSDK、IndustrialCameraPlugin、ProcessFlowPlugin |

编码约定：UTF-8 BOM + CRLF（与现有 `.filters` 一致）。仅改 `.vcxproj.filters`（必要时补缺失文件），**不改编译项、不搬物理文件**。

## 2. 原始需求

整理 `CloudSim.sln` 内**所有工程**的筛选器：在现有 `inc` / `src` 基础上，按**功能点**做更细致划分。

## 3. 边界确认

### 纳入

- 更新 / 新建各工程 `*.vcxproj.filters`
- 按功能点增加筛选器子树，并把 `ClInclude` / `ClCompile` / `QtMoc` / `None` 等条目归入对应 Filter
- 为缺失 `.filters` 的 SLN 工程补齐
- 保留现有 `inc` / `src` 顶层约定（功能夹作为其子级，或与磁盘路径对齐）

### 不纳入

- 不移动、不重命名磁盘上的源文件目录
- 不修改 `.vcxproj` 编译列表 / 依赖 / 预处理器
- 不调整 Solution Folder（`.sln` 工程分组）
- 不改业务代码与注释
- 不处理 SLN 外工程（如 `HelloAiPlugin`、`CloudSimPluginHost`），除非另决策

## 4. 需求理解

痛点：大工程（Builtins ~123、RobotWidget ~72、Widget ~61、GeometryAlgorithm ~87、RobotScene ~92、Data ~48）在 Solution Explorer 中扁平难找；磁盘上已有的 `ops/`、`sim/`、`calib/` 等功能目录在筛选器里被抹平。

目标形态示例（Builtins）：

```
inc\
src\                          # 注册表 / 公共数学
ops\
  ops\Approach
  ops\AssignBlend
  ...
```

目标形态示例（Widget，磁盘扁平、按文件名功能归类）：

```
inc\
  MainWindow
  OsgWidget
  BackendTree
  PickOperations
  Infrastructure
src\
  （镜像同名功能夹）
```

## 5. 疑问澄清（待决策）

### Q1. 划分策略（最高优先级）

| 方案 | 含义 | 影响 |
|------|------|------|
| **A. 磁盘优先 + 名称补全**（推荐） | 有子目录则镜像到筛选器；扁平工程按文件名前缀/职责聚类 | 与仓库布局一致，大工程收益最大 |
| **B. 纯功能名（中文）** | 如 `主窗口`、`轨迹算子`、`碰撞` | 可读性强，但与路径/英文代码不一致 |
| **C. 纯功能名（英文）** | 如 `MainWindow`、`TrajectoryOps` | 与代码标识一致；磁盘子目录仍尽量镜像 |

### Q2. 与 `inc`/`src` 的层级关系

| 方案 | 形态 | 说明 |
|------|------|------|
| **A. 功能夹挂在 inc/src 下**（推荐） | `inc\MainWindow`、`src\MainWindow` | 头/源仍先按类型分，再按功能 |
| **B. 功能夹与 inc/src 并列** | `MainWindow` 下再分头/源，或头源混放 | 更偏功能导航，打破现有顶层习惯 |
| **C. ops 等磁盘树顶层保留** | `ops\Approach` 与 `inc`/`src` 并列（Builtins 特例） | 与磁盘一致；其余工程仍用 A |

### Q3. 执行范围

| 方案 | 范围 |
|------|------|
| **A. 全部 39 工程** | 小工程也按功能细分（可能只有 1–2 个功能夹） |
| **B. 大工程优先**（推荐） | 先做：Builtins、GeometryAlgorithm、Widget、RobotWidget、Data、RobotScene、CloudSimHost、PointCloudAlgorithm、ProcessFlow、IndustrialCameraSDK；其余小工程仅补缺失 filters 或轻量细分 |
| **C. 仅磁盘已有子目录的工程** | 只镜像路径，不按文件名做功能聚类 |

### Q4. 缺失 `.filters` 的 5 个工程

是否一并补齐？**推荐：是**（CollisionAlgorithm、RobotCommSDK、IndustrialCameraSDK、IndustrialCameraPlugin、ProcessFlowPlugin）。

### Q5. 跨工程引用头（如 Widget 引用 Host 的头）

| 方案 | 说明 |
|------|------|
| **A.** 放入本工程对应功能夹（如 Document/Host） | |
| **B.** 单独 `External` / `HostRefs` 筛选器 | 更清晰标明非本工程源 |
| **C.** 保持原样挂在 `inc` | 改动最小 |

## 6. 建议默认（待你确认后写入 CONSENSUS）

1. **Q1 = A**（磁盘优先 + 名称补全）
2. **Q2 = A + C**（功能夹在 `inc`/`src` 下；Builtins 的 `ops\` 顶层镜像磁盘）
3. **Q3 = B**（大工程优先，小工程轻量/补缺）
4. **Q4 = 是**
5. **Q5 = B**（跨工程头进 `External`）

确认后进入 Architect → 产出各工程功能筛选器映射表与执行任务拆分。
