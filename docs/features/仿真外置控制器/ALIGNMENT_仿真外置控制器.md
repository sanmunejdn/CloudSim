# ALIGNMENT — 仿真外置控制器

> **协议权威**：[`CONSENSUS_仿真外置控制器.md`](CONSENSUS_仿真外置控制器.md)（protocolVer=1 已冻结）。本文只对齐需求与上下文，**不**改 schema / 字段名 / 端口。
>
> **文档权威**：活文档 `CloudSim/docs` > 归档 zip（见 [`ARCHIVE_ZIP_LOCATION.txt`](../../ARCHIVE_ZIP_LOCATION.txt)）。

## 1. 原始需求

希望具备 **Webots 风格的外置控制器**：用户控制逻辑跑在**独立进程**（Python / C++ 可执行程序），经本机回环 TCP 驱动 CloudSim 仿真关节，而不是把控制循环嵌进 Host。

典型期望：

1. 控制器 `connect` → 握手 → 按仿真步 `STEP` 下发目标关节角。
2. Host 在仿真 tick 应用目标并回传实际角 / 仿真时间。
3. 控制器崩溃或断开时，仿真 Host **不拖垮**。
4. 默认路径保持现有示教 / 指令回放不变。

## 2. 任务边界

| 纳入 MVP | 不纳入本波 |
|----------|------------|
| localhost TCP + 一行一 JSON 帧 | 命名管道、ROS bridge |
| C++ Client SDK（`CloudSimControllerSDK`） | Host 内嵌控制循环 |
| 最小 Python 客户端示例 | `<extern>` 双通道、Sim/Real 双后端 |
| `ControlContext` + Host `ControllerManager` | 大图 / 点云通道 |
| `ExternalController` 模式与内置回放互斥 | 平行 pose JSON（位姿仍走 `worldMatrix`） |
| Debug\|x64 + Release\|x64 双配置构建 | 私改 `OutDir` |

控制量约定：**关节角 rad**；场景位姿仍走 `worldMatrix`。

## 3. CloudSim 现状上下文

### 3.1 仿真 tick / 指令回放

- 桌面与 Headless 均已有播放 tick：`RobotProgramExecutor`（`src/Robot/RobotScene/`）。
- UI 侧由 `RobotSimulationController` 驱动定时器（典型间隔与 CONSENSUS 中 `simDtMs=16` 对齐）。
- Headless 侧：`HeadlessRobotPlaybackBridge` 复用同一执行器模型。

**插入点（共识）**：新增 `ControlContext` 承载外部 `pendingTargets` / 传感器快照；在 `ExternalController` 模式下 tick 走「外部 STEP → Context → `applyJointAnglesRad` / PoseSink」，与 `RobotProgramExecutor` 指令回放**互斥**（暂停或互斥，默认关闭外置模式）。

### 3.2 RobotCommSDK（真机）vs 外置控制器（仿真）

| | RobotCommSDK | CloudSimControllerSDK（本专题） |
|--|--------------|--------------------------------|
| 端口 | **19610** | **19620**（冻结） |
| 角色 | 真机反馈（Client → Bridge → 厂商） | 仿真外置控制（Client ↔ Host Manager） |
| 典型命令 | ping / connect / get_feedback | HELLO / STEP / GOODBYE |
| 进程关系 | 控制器连 Bridge | 控制器连 Host 内 Manager |

禁止混用端口与消息字段。帧风格对齐 RobotComm：**一行一条 UTF-8 JSON + `\n`**，便于调试。

### 3.3 尚无的能力（对齐前缺口）

- 无 `CloudSimControllerClient` / `ControllerManager`。
- 无仿真侧 `ExternalController` 模式开关与 listen `127.0.0.1:19620`。
- 无独立于 RobotComm 的仿真控制协议文档（现已由 CONSENSUS 冻结）。

## 4. 模块落点（与 CONSENSUS §5 一致）

| 组件 | 路径 |
|------|------|
| ControlContext | `src/Robot/RobotScene/` |
| Client DLL | `src/Plugins/CloudSimControllerSDK/` |
| Manager | `src/Host/CloudSimHost/` |
| UI 钩子 | `src/UI/RobotWidget/`（独立 cpp，避免与现有大文件冲突） |
| Python | `resource/Python/ControllerPython/` |

构建产物：Debug\|x64 → `bin\x64d\`；Release\|x64 → `bin\x64\`。禁止私改 `OutDir`。

## 5. 开放问题 → CONSENSUS 已决

| # | 曾存疑点 | CONSENSUS 决议 |
|---|----------|----------------|
| Q1 | Transport：命名管道 vs TCP？ | localhost TCP `127.0.0.1:19620` |
| Q2 | 与 RobotComm 是否共用端口/字段？ | **否**；19610 vs 19620，消息集分离 |
| Q3 | 控制量用关节还是 pose？ | 关节角 rad；不引入平行 pose JSON |
| Q4 | 是否改内置示教默认路径？ | 否；`ExternalController` 默认关，与回放互斥 |
| Q5 | `simDtMs` / `STEP.dtMs`？ | 默认 16；dtMs 应为倍数（MVP 可仅取本帧目标角） |
| Q6 | 多连接 / 多机器人？ | MVP：单连接，一机器人一会话 |
| Q7 | Client 崩溃行为？ | Host 关 socket，仿真继续 |
| Q8 | 协议版本？ | `protocolVer=1`，字段冻结，后续 Agent 只许引用 |

## 6. 成功标准（对齐 CONSENSUS §7）

1. Python 连 `127.0.0.1:19620`，`STEP` 循环能改变仿真关节角。
2. 控制器崩溃不拖垮 Host。
3. Debug + Release 均通过。
4. 默认路径不影响现有指令回放。
