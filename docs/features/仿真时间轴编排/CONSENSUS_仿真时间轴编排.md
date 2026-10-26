# CONSENSUS — 仿真时间轴编排

## 1. 锁定决策

| 项 | 约定 |
|----|------|
| MVP 范围 | **C**：编排编辑 + 播放头 scrub + 物料流**关键帧**（显隐/位姿，**无物理**） |
| 产品形态 | Study 仿真 Dock **独立页「时间轴」**（顶层可见，不藏在仅 Robot 子 Tab） |
| 时钟语义 | 共享 `LogicalClock` 连续秒；时间轴为 **TimeBased 排程 UI** |
| 与 EventBased | **互斥运行**：`EventBased` / ExternalPlc 会话中时间轴页只读；切事件模式前须 Stop |
| 与指令页 Run | 时间轴提供 **独立 Run/Stop**；编译为 Kernel Lane / 定时动作，**不**改写用户 catalog 程序正文（可引用 programId / poseId） |
| Seek 方案 | MVP：**PlaybackTape**（按固定采样记录姿态/信号/物料关键帧快照），scrub 读磁带；**不做**全量可逆 tick / 物理回滚 |
| 物料流 | 轨类型 `MaterialClip`：绑定 document 对象 id；关键帧字段 `visible` + `worldPose`（或相对父节点）；无 Appearance/刚体 |
| 信号 | 轨类型 `SignalClip`：bool 窗/脉冲；时间轴可 `setDo`；与程序同写冲突 → **时间轴会话内时间轴优先**，记警告 |
| 通讯 | 复用现有 Io / Kernel；不新建 PLC 权威 |
| 工程 JSON | 侧车键 `timelineStudy`；缺省空 = 无时间轴 |
| 不做 | ProcessFlow DES 合并、Appearance 物理、网页对等、NLE 音频轨、替换指令树编辑器 |

验收一句话：**同一时间尺上可拖机器人段、设备姿态段、信号窗、物料关键帧；Run 按排程驱动 3D；拖播放头可 scrub 已录/可采样状态。**

## 2. 验收标准

| ID | 标准 |
|----|------|
| T1 | Dock 有「时间轴」页；可增删轨与片段，拖改 `tStart`/`tEnd` |
| T2 | Robot 片段引用 catalog `programId`；Run 到点启动对应 Lane |
| T3 | Device 片段引用 `poseId`；到点 `goNamedPose`（经现有 player/ticker） |
| T4 | Signal 片段到点写 DO；DI 轨可显示期望/采样（MVP 至少 DO 写出） |
| T5 | Material 关键帧：拖时间戳改 `t`；scrub/Run 时物体显隐与位姿插值（线性即可） |
| T6 | 播放头拖动：在已录制区间内 3D 跟随；未录制区间提示需先 Play 或预采样 |
| T7 | EventBased 运行中打开时间轴 → 只读 + 提示；TimeBased 指令 Run 与时间轴 Run **互斥**（一方进行中另一方拒绝） |
| T8 | 保存/打开工程恢复 `timelineStudy` |
| T9 | RobotScene / RobotWidget / Widget Debug\|x64 + Release\|x64 通过 |

## 3. 技术约束

- UI：`RobotWidget` 新页；模型可放 Widget 或 RobotScene 纯数据（优先 RobotWidget + 薄 Scheduler，Kernel 少侵）
- 禁止 DevicePoseMotionPlayer 独立 QTimer 写场景（与统一内核一致）
- PlaybackTape 采样间隔默认与 playback timer 一致（~16–50ms 可配）
- 旧工程无 `timelineStudy` 必须可打开

## 4. 任务边界

见 `TASK_仿真时间轴编排.md`。B+「PLC 位直接启姿态」不在本共识。
