# 指令 / IK / 轨迹四层

1. **Instruction（持久化）**：仅 TCP（pose/euler + 变换扩展）+ 运动参数 + 轴配置；禁止关节 CSV。存储位姿恒为 **基座（P0）工具原点**；User 系仅显示层。
2. **PlanRequest / IkRequest**：临时；`IkSeedPolicy` = `FromCurrentPose` | `FromInstruction`；种子由 UI 解析后注入 `prepareInstructionIkContext`。求解侧 `IkSolveRequest` / `IkSolveResult`；`IkSolveMetrics` 只进诊断日志（不写 plan.summary，避免被当失败原因误判 Budget），Budget 类失败时附 Run 警告；失败原因优先取 planner `err`。
3. **IK**：球形腕 `solveAll` 多腕支入池（外轴激活时跳过，走耦合数值路径）→ 否则 DLS。LINE/ARC 样点失败时**局部加密**（段内 0.25/0.5/0.75）再全局加倍。`testdata/ik_golden/*.json` + `runIkGoldenSelfTest` 校验肘/支约定（可按包名加真机位姿）。
4. **Ruckig（PTP）**：IK 成功后生成 `jointTrajectoryRad`；LINE/ARC 笛卡尔采样。

UI 只组种子并调用 `plan`；连续性/构型门以 Planner `IkSelectionPolicy` 为准。

详见 `docs/ARCHIVE_ZIP_LOCATION.txt`。
