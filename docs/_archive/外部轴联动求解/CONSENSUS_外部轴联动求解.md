# CONSENSUS：外部轴联动求解

## 需求描述

为机器人实例配置外部轴（一期单自由度地轨），持久化到工程；仅在配置启用时，`ExternalAxisSearch` 与联立 IK 将外轴纳入求解。同步优化 MDH 解析雅可比与 URDF 棱柱步长限幅。

## 验收标准

1. Dock 有「外部轴」Tab，可添加/启用/编辑一条地轨并保存重开仍在
2. 未配置时 ExternalAxisSearch 不写假 `rail_joint`，IK 与改前一致
3. 启用后 Search 在行程内给出 `q_e`，更新 `reachable` 与 `ctx.externalAxes`
4. MDH 解析 J 与数值 J 列误差 &lt; 1e-5（单测或自检函数）
5. 联立路径棱柱 `dq` 按 mm 限幅，不被 ±0.2 rad 误伤
6. 相关工程可编译
7. Run/点云播放时臂与地轨连续运动（非瞬移）；`jointTrajectoryRad` 保留供插帧

## 技术方案

| 层 | 方案 |
|----|------|
| 模型 | `RobotExternal::RobotExternalAxisConfigSet` 挂 `HierarchicalRobotInstance` |
| IO | `robotKinematicsInstances[].externalAxes` |
| UI | `RobotExternalAxisSettingsWidget` + Dock Tab |
| 门禁 | `hasEnabledExternalAxes(set)` |
| Search | ctx 注入配置 + `IExternalAxisSearchService`；无服务/无配置则跳过 |
| 联立 | TeachIk / Kinematics 可选外轴 DOF |
| 存储 | P0 + `externalAxisQMm`；FK 合成 P_eff |
| 播放 | 臂轨迹插帧；外轴按段进度 lerp；时长含 \|Δqe\| |
| Kinematics | 闭式 MDH、解析位置 J、`DhRow::isPrismatic` |

## 约束

- 不破坏 `DhRow` 默认 revolute ABI（新字段默认 false）
- 轨迹 Op 不重复编辑行程
- 主 IK 仍走 URDF；DH 为回退与单测
