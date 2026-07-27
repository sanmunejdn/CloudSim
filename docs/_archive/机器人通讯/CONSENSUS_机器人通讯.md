# CONSENSUS — 机器人通讯

## 需求与验收

1. `scripts/fetch_hslcommunication.ps1` 执行成功，`bin/SDK/HslCommunication-12.9.1/` 含 `HslCommunication.dll` 及 `Newtonsoft.Json.dll`
2. `RobotCommBridge` 启动（localhost TCP JSON）；CloudSim 经 SDK `connect` 到 Bridge（非直连机器人）
3. 三品牌 Adapter 可连：`AbbHslAdapter` / `FanucHslAdapter` / `KukaHslAdapter`（KUKA 优先 `KukaTcpNet`）
4. `get_feedback` 返回非空 `jointRad[]` 与/或 `toolPoseInBase`（positionMm + eulerDeg ZYX）
5. 镜像开启后场景关节随反馈更新（`doc->applyJointAnglesRad`）；断开后停止写场景
6. Dock 新增 Tab **「机器人通讯」**：品牌 / IP / 连接 / 镜像 / 采样率 / 关节与 TCP 只读显示 / 状态日志
7. 失败有中文提示，不崩溃；无真机时可用 HSL Demo Server 联调

## 技术方案

```text
RobotCommPageWidget → RobotSimulationController
  → RobotCommSDK (IRobotMotionClient)
    → localhost TCP JSON → RobotCommBridge (C#)
      → HslCommunication.dll → ABB / Fanuc / KUKA
  → applyJointAnglesRad → 数字孪生
```

| 组件 | 说明 |
|------|------|
| `RobotCommSDK` | `IRobotMotionClient`、DTO、TCP 客户端；对齐 `PlcCommSDK` |
| `RobotCommBridge` | 授权、HSL 客户端池、品牌 Adapter、JSON 协议翻译 |
| `RobotCommPageWidget` | Dock Tab UI；信号经 Controller 编排 |
| `fetch_hslcommunication.ps1` | NuGet 拉取 12.9.1 → `bin/SDK/` |

Bridge 命令集：`connect` / `disconnect` / `ping` / `get_state` / `get_feedback`。

## 约束

- 工具链：x64、v142、C++17、UTF-8
- 单位：关节 **rad**；位姿 **mm**；欧拉 **ZYX deg**
- 授权：`HSL_AUTH_CODE` 环境变量或本地 `hsl.auth`（**不提交 git**）；无码开发试用约 8h
- **禁止破解 / 伪造激活**；产品交付须商用授权
- 机侧依赖：ABB RWS、Fanuc 以太网接口、KUKA TCP 程序 / KukavarProxy
- 一期不做 EGM/RSI；HSL 路径适合监控与低频同步（非 4ms 闭环）
- SDK **不**直连机器人；**不**把厂商 Socket 挂到 `IRobotBackendPoseSink`
