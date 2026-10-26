# ACCEPTANCE — 阶段 B：外接 PLC

| ID | 期望 | 状态 |
|----|------|------|
| B1 | EventBased+LocalCee 行为与 A 一致 | 代理通过（Kernel 仅 LocalCee 调 evaluate；权威缺省 localCee） |
| B2 | 映射 in：PLC→仿真 DI，Wait 可读 | 代理通过（`onAfterIoPhase` forced DI + 工作线程 readTag；完整 OpenPLC 回路建议人工点验） |
| B3 | 映射 out：仿真 DO→PLC | 代理通过（读 sink DO → writeTag；OpenPLC 线圈可读建议人工点验） |
| B4 | 断线警告且仿真继续 | 代理通过（sync 失败 `plcWarning` + 清 connected；session 不 abort） |
| B5 | Run 中拒切权威；Stop 后可切 | 代理通过（`setEventLogicAuthority` 会话守卫；Stop 断开桥） |
| B6 | Debug+Release x64 相关工程通过 | 通过（RobotScene / RobotWidget / Widget） |

说明：B2/B3 的 OpenPLC Runtime 真机回路仍建议人工复扫；上表为代码路径 + 双配置编译代理门闩。BugTracker #69。
