# ALIGNMENT — 机器人通讯

## 原始需求

在 CloudSim 内建立与真实机器人的在线通讯：读取关节角与 TCP 末端位姿，驱动仿真场景数字孪生镜像；适配 ABB / Fanuc / KUKA 等常见品牌。

## 项目上下文

- CloudSim 机器人栈（`RobotScene` / `RobotWidget`）**仿真优先**；现有真实机器人能力为离线品牌导出、相机侧 `GET_POSE` TCP，无在线关节/位姿总线
- 技术栈：Qt + OSG + C++17；参考形态：`PlcCommSDK` + `PlcCommPlugin`、工业相机 `IndustrialCameraSDK`
- 驱动参考：HslCommunication **12.9.1 商业 NuGet**（Demo 在本地 `HslCommunication-master`）；**不采用 Community 版**（缺 ABB RWS / Fanuc 以太网 / `KukaTcpNet`）
- 接入形态：**C# RobotCommBridge**（托管 HSL）+ **C++ RobotCommSDK**（统一接口）；主进程无 CLR
- UI：在 `RobotSimulationDockWidget` 现有 Dock 内**新增 Tab「机器人通讯」**，不另开独立设备页

## 边界确认

| 纳入（MVP） | 不纳入（一期） |
|-------------|----------------|
| 连接 + 只读反馈（关节角 / TCP 位姿） | 在线 Run 驱动真机 |
| 数字孪生镜像（`applyJointAnglesRad`） | EGM / RSI 硬实时闭环 |
| ABB / Fanuc / KUKA 三品牌 Adapter | 全品牌并行（EFORT / Estun 等二期） |
| Bridge localhost TCP JSON | SDK 直连厂商 Socket |
| `scripts/fetch_hslcommunication.ps1` 安装 DLL | 源码入库 HSL |
| Dock Tab 连接 / 状态 / 镜像 / 采样率 | 点动 / moveJ / moveL 写运动 |

## 需求理解

1. SDK 只连 Bridge，Bridge 再经 HSL 连机器人；CloudSim 不直接引用 `HslCommunication.dll`
2. 反馈单位对齐空间契约：关节 **rad**；末端 **mm + ZYX 欧拉 deg**（与 `RobotRigidFrame` / 指令 `pose` 一致）
3. 镜像开启时 Controller 周期性 `get_feedback` → 写场景；断开即停写
4. HSL 授权：开发可选 `HSL_AUTH_CODE` 环境变量；无码不阻断启动（约 8h 试用限制）

## 疑问澄清（已决策）

| 问题 | 决策 |
|------|------|
| MVP 能力范围 | 只读反馈 + 数字孪生镜像 |
| 品牌优先级 | ABB + Fanuc + KUKA 并行骨架 |
| HSL 版本 | 商业 **12.9.1** NuGet，非 Community |
| 接入方式 | C# Bridge + C++ SDK（非整进程 CLR） |
| KUKA 协议 | 优先 `KukaTcpNet`；`KukaAvarProxyNet` 备选 |
| UI 入口 | `RobotSimulationDockWidget` 新 Tab |
| 授权策略 | 有 `HSL_AUTH_CODE` 则激活；无码仅警告；**不破解** |
