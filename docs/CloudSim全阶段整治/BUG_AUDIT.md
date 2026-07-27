# BUG_AUDIT — CloudSim 全阶段整治

四路只读审计合并。非 Top3 条目本轮不改代码。

## Top3 修复范围

| Top3 | 本轮采纳 | 延后（仅记录） |
|------|----------|----------------|
| T1 轨迹编辑 | B1–B5（MIME 门禁、同排落点、去掉掩盖、索引钳制） | B6–B8 死 API 删除可顺手做 |
| T2 Run/外轴 | R1、R2、R8（静默臂-only、擦除外轴字段、timer 空指针） | R3–R7（多样本外轴对齐、示教 clear、万级轴、维数跳过） |
| T3 Mesh/catch | Mesh Face/Polyline 入口拒绝未实现 mode；C01–C04 加日志 | C05–C11 DEFER |

---

## A — Robot Run / 外轴 / 插帧

| ID | Sev | File:line | 摘要 | Top3 |
|----|-----|-----------|------|------|
| R1 | P0 | RobotInstructionController.cpp:1939+ | 外轴 IK 失败后回落 DH/legacy，静默臂-only | T2 修 |
| R2 | P0 | RobotSimulationController.cpp:638-643 | `!hasExternalAxisQ` 时 erase 指令外轴字段 | T2 修 |
| R3 | P1 | RobotInstructionController LINE 采样 | 外轴只保留终点 qe，中段不一致 | 记录 |
| R4 | P1 | RobotSimulationController taught 路径 | 显式 `jointTrajectoryRad.clear()` | 记录 |
| R5 | P1 | RobotProgramExecutor.cpp:197 | 维数不匹配仍推进段 | 记录 |
| R6 | P1 | 万级 pose axes 无 stride | 卡顿 | 记录 |
| R7 | P2 | applyExternalAxisFromPlan 空目标 | 滑轨停旧值 | 记录 |
| R8 | P2 | playbackTimer start/stop | 空指针崩溃风险 | T2 修 |

## B — TrajectoryEdit 拖放 / m_ops

| ID | Sev | File:line | 摘要 | Top3 |
|----|-----|-----------|------|------|
| B1 | P0 | TrajectoryPipelineListWidget dragEnter | 非 kMimeType 仍 accept | T1 修 |
| B2 | P0 | dropEvent 同排 | 落点同排后未 return → 重复插入 | T1 修 |
| B3 | P0 | flushPipelineToSession 对齐 | 掩盖根因 | T1：根因修后弱化 |
| B4 | P1 | insert 用 count() | 可越界 m_ops | T1 修 |
| B5 | P1 | 菜单用视觉 count | 失步时错选 | T1 修 |
| B6 | P2 | refreshScopeFieldVisibility 空桩 | 无调用方 | T1 删死代码 |
| B7 | P2 | refreshParamPanelForKind 空桩 | 无调用方 | T1 删死代码 |

## C — MeshDiscretize

| ID | Sev | 摘要 | Top3 |
|----|-----|------|------|
| M1 | High | ProfileSweepMesh/RemeshSoup/PointCloudSurface 未实现 | T3：统一 isImplemented + Face/Polyline 入口拒绝 |
| M2 | Med | Types.h/文档漏标 ProfileSweepMesh | T3 补注 |
| M3 | Info | UI/插件枚举已不含 stub | 无需 UI 改 |

## D — 空 catch / I/O

| ID | Class | File:line | Top3 |
|----|-------|-----------|------|
| C01 | MUST_LOG | MainWindowProjectIo.cpp:415 robotCollision | T3 |
| C02 | MUST_LOG | RobotProjectKinematicsRestore.cpp:209 coordinateFrames | T3 |
| C03 | MUST_LOG | RobotProjectKinematicsRestore.cpp:247 externalAxes | T3 |
| C04 | MUST_LOG | PluginDelegatedBackend.cpp:55 propertyRowsJson | T3 |
| C05–C11 | DEFER | CSV/stoi/算法回退等 | 记录 |
