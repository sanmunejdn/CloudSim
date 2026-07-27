# ACCEPTANCE：架构边界收口（Sprint H）

> 日期：2026-07-21  
> 范围：DocumentPage FK 绑定内部存储 Mat4；`backend()` 策略闭环；Assess 文档

## 验收项

| ID | 项 | 证据 | 结果 |
|----|----|------|------|
| H1 | HierarchicalRobotInstance FK/outer/base 为 Mat4 | `DocumentPage.h` 字段类型 | **通过** |
| H2 | 聚合缓存 Mat4 | `m_robotFkMeshWorldT0` / `m_robotOuterWorldAtBind` | **通过** |
| H3 | 导入/恢复调用方转换 | `UrdfRobotImport` / `RobotProjectKinematicsRestore` → `coreMat4FromOsgMatrix` | **通过** |
| H4 | 基座默认单位阵 | `appendHierarchical…` 写入 `identityMat4()`（避免 Mat4{} 全零） | **通过** |
| H5 | `backend()` 策略文档化 | DEVELOPER_GUIDE + `DocumentPage::backend` 注释 + §11 | **通过** |
| H6 | 编译 | `CloudSim.sln` `/t:RobotScene;CloudSimHost;RobotWidget;Widget` Debug\|x64 | **通过** |

## 已知残余（不阻塞本轮闭环）

- 关节场景句柄仍为 `osg::MatrixTransform*`（per-link 可为空）
- `DocumentPage::backend()` 物理穿透仍在（白名单策略，非本轮删除）
- `RobotSimulationController` → Host、IRenderView 全面替代 → 长期

## 手工回归建议

- URDF 导入后 per-link FK / 回放
- `.pcp` 打开后 kinematics 恢复（basePlacement + outer bind）
- 基座放置 / Follow

## 变更摘要

- DocumentPage 内部 FK 绑定存储：`osg::Matrixd` → `core::Mat4`
- Host 导入/恢复在边界转换为 Mat4 再绑定
- `backend()` 明确为存量白名单；新代码走 `data()`
