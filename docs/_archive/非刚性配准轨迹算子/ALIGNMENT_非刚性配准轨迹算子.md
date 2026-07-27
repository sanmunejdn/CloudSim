# ALIGNMENT — 非刚性配准轨迹算子

## 原始需求

在轨迹流水线中新增算子：参考 HPL `HPLTPNonRigidRegistrationStrategy`，先绑定轨迹点到源几何，再按 SPARE 非刚性配准结果纠正轨迹位姿。SPARE 已在 PointCloud/Data 层实现，本任务不重写算法。

## 边界

- 源/目标：点云或 mesh（2×2）；不支持 BREP
- 同步 SPARE；不写回场景 backend；不做 HPL `_deformed.bin` 导出
- 仅更新 `poseMm`，不改 `eulerDeg`

## 集成

- Builtins 四件套 + `INonRigidTrajectoryWarp` ctx 注入（同 ProjectToGeometry）
- RobotScene 适配器调用 `point_cloud_backend_ops::nonRigidRegister*Spare`
