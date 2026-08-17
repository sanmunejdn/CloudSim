# FINAL — Robot 运动学演进

## 交付摘要

- **Workspace**：`UrdfKinematicsWorkspace` + 索引图 BFS Jacobian；联立 TeachIk 复用 scratch
- **Options**：`UrdfIkSolverOptions` / `TeachIkContext.options`
- **核拆分**：`UrdfNumericalIk::solveArmPoseDampedLeastSquares`；TeachIk / InstructionController 调用
- **DH**：标 legacy；生产空 `dhRows`
- **Oracle**：`runSelfTest` + golden JSON + `tools/pinocchio_oracle`
- **文档/图**：6A、四处 DEVELOPER_GUIDE、`diagrams/*.html`
- **filters**：RobotUrdf 按 Global/Loader/Kinematics/SelfTest

## 编译

RobotUrdf / RobotScene：**Debug|x64** 与 **Release|x64** 均已通过。
