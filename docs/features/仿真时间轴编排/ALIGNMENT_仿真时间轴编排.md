# ALIGNMENT — 仿真时间轴编排页

## 1. 项目上下文

| 项 | 现状 |
|----|------|
| Study 仿真入口 | `RobotSimulationDockWidget`：机器人 / 自定义设备 分模式 Tab；指令页 `SimulationCommandWidget` |
| 时钟 | `SimulationKernel` + `LogicalClock`：**仅前进**（Run/Stop + 倍率），**无 seek** |
| 多资源协调 | Lane / SyncBarrier / IoPhase 下一拍（统一内核已落地） |
| 事件模式 | TimeBased \| EventBased + LocalCee \| ExternalPlc（阶段 A/B）——**逻辑权威**，不是时间编排 UI |
| 信号 | `IoSignalPageWidget` + `IoSignalNetworkService`（表格式，无时间轨） |
| 物料流 / Appearance | **未落地**；CEE 文档明确排除 |
| ProcessFlow | 独立工作区 DES；**跑后** Gantt + scrub 回放 canvas，**不是** Study 3D 多轨时间轴编辑 |

缺口：缺少一张「以共享时间轴为中心」的页面，把机器人程序段、自定义设备姿态、信号边沿、（未来）物料流片段排在同一 `t` 上，并用**拖动时间戳**编辑/ scrub。

## 2. 原始需求

> 设计单独的基于时间节点的仿真页面；不只是机器人运动；自定义设备、物料流、信号等都能拖动时间戳来控制。

解读：

- **单独页面**：不塞进指令列表旁的小控件；独立 Dock Tab（或等价全宽子页）
- **时间节点**：以 `t`（秒或 tick）为第一公民，片段/关键帧可拖拽改 `tStart`/`tEnd`
- **多轨资源**：至少机器人、自定义设备、信号；物料流作为轨类型预留或 MVP 占位
- **控制**：拖时间戳 = 改排程；播放头拖动 = seek（需内核补 seek 或录制缓冲）

## 3. 边界确认（草案，待共识）

**建议在范围内（MVP）**

- 新 Tab：`时间轴`（挂在 SimulationDock，跨 Robot/Device 模式可见，或 Dock 顶层第三模式）
- 多轨视图：资源行 × 时间条；片段块可水平拖、改时长；播放头可拖
- 轨类型 MVP：`RobotProgramClip`、`DevicePoseClip`、`SignalPulse`（bool 边沿/电平窗）
- 与现有 `TimeBased` 会话衔接：时间轴「排程」编译/映射为 Kernel Lanes + 定时 setDo；或录制回放缓冲
- 工程 JSON 侧车键（如 `timelineStudy`）
- 复用 ProcessFlow Gantt **交互经验**（视觉可参考），数据模型落 Study，不并进 ProcessFlow 工作区

**建议不在范围内（MVP）**

- 把 ProcessFlow DES 时钟与 Study LogicalClock 合并
- Appearance / 刚体输送完整物理（物料流轨可先「占位 + 显隐/绑定 ID」）
- EventBased CEE / ExternalPlc 规则编辑迁入时间轴（时间轴属 **TimeBased 编排**；事件模式仍用规则表）
- 网页对等、硬实时、NLE 级音频式多层合成
- 替换指令树编辑器（时间轴是编排层，程序细节仍可在指令页编辑）

## 4. 需求理解（相对现有能力）

| 能力 | 今日 | 时间轴页要补的 |
|------|------|----------------|
| 多机并行 | Run All + Lane | 在时间上**错开/对齐**可视编辑 |
| 设备姿态 | Device 页 / CEE / Driver | 姿态段放在同一时间尺上 |
| 信号 | IO 表 + 程序 WAIT/SET_DO | 信号窗/脉冲作为可拖轨 |
| 物料流 | 无 | 轨类型预留；MVP 可不驱动物理 |
| Seek | 无 | 播放头拖动 → 需录制帧缓冲或可逆 tick（架构决策） |

**与事件模式关系：**  
时间轴 = **开环/半开环排程**（谁在何时做什么）；EventBased = **闭环联锁**。二者可并存：同一工程可有 `timelineStudy` + `ceeRules`，Run 时按 `simulationRunMode` 二选一主时钟语义（共识需锁）。

## 5. 疑问澄清（按优先级）

### P0（已锁定 2026-10-23）

用户选择 **C**：编排编辑 + 播放头 scrub + 物料流关键帧（显隐/位姿，无物理）。

其余运行关系冻结见 `CONSENSUS_仿真时间轴编排.md`：时间轴独立 Run；与指令 Run / EventBased 互斥；Seek 用 PlaybackTape。

~~原选项 A/B/物料 A·B·C 列表已归档。~~

### P1

4. 挂载位置：SimulationDock **新顶层 Tab** vs **Robot 模式下第 8 个 Tab** vs **独立 Dock**  
5. 时间单位：连续秒（LogicalClock）vs 离散帧（固定 Hz）  
6. 信号轨：只画 DI/DO 期望波形，是否允许时间轴直接 `setDo`（与程序/CEE 写同一信号的冲突策略）

## 6. 初步落点（未冻结）

```
RobotWidget
  SimulationTimelinePageWidget   // 多轨 UI
  SimulationTimelineModel        // clips / tracks / playhead
RobotScene（或 Widget 侧）
  TimelineScheduler              // clips → Lane start / setDo at t
  LogicalClock::seek 或 PlaybackTape（录制缓冲）
工程 JSON: timelineStudy { tracks[], clips[], durationSec }
```

参考：`ProcessFlowGanttWidget` / `ProcessFlowReportPanel::m_playSlider`（交互），数据勿绑 DES。
