# ALIGNMENT：外部轴联动求解

## 原始需求

1. 分析并优化机器人求解器（RobotKinematics / URDF 数值 IK）可优化点
2. 支持外部轴（一期地轨）联动求解
3. 在机器人对象上提供外部轴配置 UI；**仅配置并启用后**才进行联动求解

## 项目上下文

- `RobotKinematics`：MDH FK + 位置 DLS IK（回退路径）
- 主路径：`RobotTeachIk` / URDF 数值 IK
- 轨迹 Op `ExternalAxisSearch` 为 Phase-3 占位（假 `rail_joint`）
- 坐标系配置模式：`HierarchicalRobotInstance` + Dock Tab + 工程 JSON

## 边界

**在范围内**：外轴对象模型与持久化、仿真 Dock「外部轴」UI、配置门禁、地轨 1D 搜索、可选联立微调、解析雅可比/棱柱限幅、文档

**不在范围内**：变位机多轴同步、闭链、解析 6R、DH 姿态 IK、Axis 控制滑条暴露外轴（可后续）、纯虚拟几何轨编辑器

## 已确认决策

1. 外轴是机器人实例属性，不是轨迹 Op 参数
2. 门禁：`enabled && axes 非空` 才联动；未配置时 Search 为 no-op，不写假 rail
3. 一期仅 LinearRail（数据结构预留 Turntable）
4. 运动学效果：沿配置轴平移基座等价（目标在基座系下减去 `q_e * axis`），快照写入 `jointName`
5. 绑定关节名：下拉候选 + 可编辑；无匹配 URDF 关节时可仍启用（虚拟地轨），但提示用户
