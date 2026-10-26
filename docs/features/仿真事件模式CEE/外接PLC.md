# DESIGN — 阶段 B：外接 PLC

## 时序（ExternalPlc）

1. Lane step / deviceTicker / latch+swap
2. 跳过 CeeEvaluator
3. Controller 调 `ExternalPlcBridge::onAfterIoPhase`：应用缓存 PLC→forced DI；投递 DO→PLC 任务
4. CollisionGuard

## 组件

- `EventLogicAuthority`（RobotScene）
- `ExternalPlcBridge` + Worker 线程（RobotWidget，链 PlcCommSDK）
- UI：权威下拉、连接参数、映射表
