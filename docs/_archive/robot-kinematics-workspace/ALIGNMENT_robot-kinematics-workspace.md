# ALIGNMENT — Robot 运动学演进（Workspace / URDF / Options / Oracle / 核拆分）

## 原始需求

对照 dynibo 思路，在 CloudSim 内落地：

1. Workspace 零分配 → TeachIk / Urdf FK 热路径缓冲池（拖动优先）
2. URDF 树唯一模型 → 巩固 RobotUrdf，淡化 DH 回退
3. 完整位姿 IK + Options → 整理容差/阻尼/步长
4. J̇ / 动力学 → 暂缓
5. Pinocchio 级对照 → RobotUrdf SelfTest（有限差分 + golden）
6. 纯运动学核从 RobotScene 再拆一层（仍可 C++）

## 项目上下文

| 模块 | 现状角色 |
|------|----------|
| RobotUrdf | URDF 解析缓存、FK、几何雅可比 |
| RobotScene / TeachIk | 位姿 DLS、外轴、联立拖动代价 |
| RobotKinematics | DH 位置 IK（生产路径空 dhRows）、圆弧几何 |
| RobotWidget | TCP 拖动节流 / chase / 台阶 |

## 边界确认

**在范围内：** Workspace、索引 BFS、Options、DH 淡化文档与路径切断、SelfTest、DLS 下沉 RobotUrdf、filters 按功能、开发文档与 diagram-design 框架图。

**不在范围内：** J̇、RNEA/重力、dynibo/Rust 替换、Pinocchio MSVC 依赖、字面 C ABI。

## 疑问澄清（已决策）

| 项 | 决策 |
|----|------|
| Oracle | 有限差分 + 提交 golden；离线脚本可选用 Pinocchio 重生 |
| 核拆分 | 扩 RobotUrdf，默认不新建工程 |
| 图皮肤 | diagram-design 默认 skin |
| filters | 按功能分子夹（Kinematics / SelfTest / Loader / Global） |
