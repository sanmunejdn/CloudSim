# TASK — 非刚性配准轨迹算子

1. 类型与 parse → 依赖无
2. INonRigidTrajectoryWarp + Engine 注入 → 依赖 1
3. RobotScene 适配器 → 依赖 2
4. Builtins 四件套 + 注册 → 依赖 1,2
5. DEVELOPER_GUIDE + 验收文档 → 依赖 4
