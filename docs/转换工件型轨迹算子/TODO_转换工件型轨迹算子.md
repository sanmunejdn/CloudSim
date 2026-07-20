# TODO — 转换工件型轨迹算子

1. **手工验收**：x64 编译 Builtins + RobotScene + RobotWidget → 拖入「转换工件型」→ 设外部 TCP → Preview/Apply（T-WH1/2/5）
2. **场景前提**：执行前机器人需有可捕获的当前 TCP（URDF FK / 场景法兰）；否则会报「缺少当前机器人 TCP 参考位姿」
3. **后续（非本任务）**：法兰→工件标定（当前单位阵）；若需世界系参考位姿再扩展注入
4. **配置**：无需额外 `.env`；JSON 资源路径 `RobotScene/resource/trajectory/ops/ToWorkpieceInHand.json`
