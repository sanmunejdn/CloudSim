# CONSENSUS — 仿真事件模式（薄 CEE）阶段 A

## 1. 锁定决策

| 项 | 约定 |
|----|------|
| 模式 | `timeBased`（默认）\| `eventBased` |
| EventBased 逻辑权威 | 仅薄 CEE；禁用 `DevicePoseSignalDriver` |
| 运动权威 | `SimulationKernel` Lane + `IDeviceMotionTicker` |
| 外置控制器 | 与 Run 互斥（两种模式均拒绝） |
| CEE 挂点 | `swapAll()` 之后；写出 DO 下拍才进 DI |
| EventBased 空闲 | 无 Lane 仍保持 session，直至 Stop |
| 条件 | `risingEdge` \| `levelHigh` \| `levelLow` |
| 动作 | `goNamedPose` \| `startRobotProgram` \| `setDo` \| `stopLane` |
| bindings 同步 | 合并按规则 id；绑定变更可再同步 |
| Run 中改模式 | 拒绝（须先 Stop） |

## 2. 验收标准

1. TimeBased 单机 PTP/WAIT/SET_DO 与改前等价
2. EventBased：DI↑ → goNamedPose 触发一次；切回 TimeBased 后 Driver 可用
3. 机 SET_DO → wire → 设备 DI↑ → CEE → Done → 机 Wait（下一拍）
4. EventBased + 外置控制器：Run 拒绝
5. Run 中改模式：拒绝
6. 保存/打开恢复 `simulationRunMode` / `ceeRules`
7. RobotScene + RobotWidget + Widget：Debug\|x64 与 Release\|x64 通过

## 3. 技术约束

- CEE 落在 **RobotScene**（无 Qt Widget）
- 启动机器人程序须抽取 Controller 辅助函数，禁止复制大段 Run 逻辑
- 工程键：`simulationRunMode`、`ceeRules`、`devicePrograms`

## 4. 任务边界

见 TASK；阶段 B（外接 PLC）不在本共识内。
