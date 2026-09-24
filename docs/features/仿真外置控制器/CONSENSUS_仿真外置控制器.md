# CONSENSUS — 仿真外置控制器 MVP

> **协议已冻结（protocolVer=1）**。P1/P2 Agent 只许引用本文件字段名，禁止私改 schema。

## 1. 需求与验收边界

| 项 | 约定 |
|----|------|
| 目标 | Webots 风格：独立控制器进程经 localhost TCP 驱动仿真关节 |
| Transport | TCP `127.0.0.1:19620`（避开 RobotComm `19610`） |
| 帧格式 | 一行一条 JSON，UTF-8，以换行结束 |
| 控制量 | 关节角 rad；不引入平行 pose JSON；场景位姿仍走 worldMatrix |
| 模式 | `ExternalController` 与内置指令回放互斥；默认关闭，现有示教不变 |
| 本波不做 | 命名管道、ROS、`<extern>`、Sim/Real 双后端、大图/点云通道 |

## 2. 与 RobotComm 边界

| | RobotCommSDK | CloudSimControllerSDK |
|--|--------------|------------------------|
| 端口 | 19610 | **19620** |
| 角色 | 真机反馈（Bridge→厂商） | 仿真外置控制器（Client↔Host） |
| 典型命令 | ping/connect/get_feedback | HELLO/STEP/GOODBYE |
| 进程 | 控制器连 Bridge | 控制器连 Host 内 Manager |

禁止混用端口与消息字段。

## 3. 连接与生命周期

1. Host 在 ExternalController 开启时 listen `127.0.0.1:19620`（单连接 MVP：一机器人一会话）。
2. Client `connect` → 发 `HELLO` → 收 `HELLO_ACK`。
3. 循环：Client 发 `STEP` → Host 在仿真 tick 应用 `targetJointRad` → 回 `STEP_REPLY`。
4. Host 结束/重置：可发 `QUIT`；Client 发 `GOODBYE` 后断开。
5. Client 崩溃：Host 关掉 socket，仿真继续；不拖垮 Host。

`simDtMs` 默认与播放定时器一致：**16**。`STEP.dtMs` 应为 `simDtMs` 的正整数倍；MVP 可忽略倍数仅取本帧目标角。

## 4. JSON Schema（冻结）

所有消息必含：`type`、`protocolVer`（值为 1）。

### 4.1 Client → Host

#### HELLO

```json
{
  "type": "HELLO",
  "protocolVer": 1,
  "robotInstanceIndex": 0
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| robotInstanceIndex | int | 文档中机器人实例下标，从 0 |

#### STEP

```json
{
  "type": "STEP",
  "protocolVer": 1,
  "dtMs": 16,
  "targetJointRad": [0.0, 0.1]
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| dtMs | int | 控制步长 ms，>0 |
| targetJointRad | number[] | 目标关节角；长度须等于 HELLO_ACK.jointCount |

#### GOODBYE

```json
{ "type": "GOODBYE", "protocolVer": 1 }
```

### 4.2 Host → Client

#### HELLO_ACK

```json
{
  "type": "HELLO_ACK",
  "protocolVer": 1,
  "simDtMs": 16,
  "jointCount": 6,
  "jointNames": ["joint_1", "joint_2"]
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| simDtMs | int | 仿真/tick 基准 ms |
| jointCount | int | 活动关节数 |
| jointNames | string[] | 可空名用 j0…；长度=jointCount |

#### STEP_REPLY

```json
{
  "type": "STEP_REPLY",
  "protocolVer": 1,
  "simTimeMs": 160,
  "actualJointRad": [0.0, 0.1]
}
```

| 字段 | 类型 | 说明 |
|------|------|------|
| simTimeMs | int | Host 侧累计仿真时间 |
| actualJointRad | number[] | 应用后实际角（通常等于目标） |

#### QUIT

```json
{ "type": "QUIT", "protocolVer": 1, "reason": "simulation_stopped" }
```

| 字段 | 类型 | 说明 |
|------|------|------|
| reason | string | 可选；人类可读 |

#### ERROR

```json
{
  "type": "ERROR",
  "protocolVer": 1,
  "code": "BAD_JOINT_COUNT",
  "message": "targetJointRad length mismatch"
}
```

### 4.3 错误码（冻结）

| code | 含义 |
|------|------|
| PROTOCOL_MISMATCH | protocolVer 不支持 |
| BAD_ROBOT_INDEX | robotInstanceIndex 无效 |
| BAD_JOINT_COUNT | 数组长度与 jointCount 不符 |
| NOT_READY | ExternalController 未开或无机器人 |
| INTERNAL | Host 内部错误 |

## 5. 模块落点

| 组件 | 路径 |
|------|------|
| ControlContext | `src/Robot/RobotScene/` |
| Client DLL | `src/Plugins/CloudSimControllerSDK/` |
| Manager | `src/Host/CloudSimHost/` |
| UI 钩子 | `src/UI/RobotWidget/`（独立 cpp） |
| Python | `resource/Python/ControllerPython/` |

## 6. 构建约定

- Debug|x64 → `bin\x64d\`；Release|x64 → `bin\x64\`
- 禁止私改 OutDir；新源须 sync filters
- 源码 UTF-8 BOM + CRLF；禁止 `#pragma once`

## 7. 成功标准（MVP）

1. Python 连 `127.0.0.1:19620`，`STEP` 循环能改变仿真关节角
2. 控制器崩溃不拖垮 Host
3. Debug+Release 均通过
4. 默认路径不影响现有指令回放
