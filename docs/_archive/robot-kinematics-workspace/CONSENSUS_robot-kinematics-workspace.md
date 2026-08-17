# CONSENSUS — Robot 运动学演进

## 需求描述

以 RobotUrdf 为 URDF 运动学真源：热路径零分配 Workspace、统一 TeachIkSolverOptions、数值 DLS 下沉至 RobotUrdf；RobotScene 仅保留外轴与联立示教策略；DH 仅作无 URDF legacy；提供 SelfTest + golden；同步 DEVELOPER_GUIDE 与框架图。

## 验收标准

1. Debug|x64 与 Release|x64 下 RobotUrdf、RobotScene 编译通过
2. TCP 拖动 / TeachIk 行为不回归（Options 默认值对齐原魔法数）
3. 有 URDF 时不走 DH 位置 IK
4. `runSelfTest`：FK golden、J 有限差分、DLS 闭环通过
5. `.vcxproj.filters` 按功能分区；新文件不在 Loader 根下混放
6. 四处 DEVELOPER_GUIDE 与 `diagrams/*.html` 可打开且描述一致

## 技术约束

- 长度 mm、关节 rad；几何 J 行主序
- Workspace / Scratch：容量不足才扩容，禁止每 iter 重建
- 拖动：Options 管求解器；UI 管 coalesce / chase / 关节台阶
- 不做 J̇、动力学、Pinocchio 链接、C ABI

## 任务边界

Phase 0 文档+图 → Phase 1 Workspace → Phase 2 Options/DH → Phase 3 SelfTest → Phase 4 DLS 下沉。
