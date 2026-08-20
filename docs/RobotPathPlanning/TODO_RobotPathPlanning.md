# TODO — RobotPathPlanning

## 运行验证（优先）

- 选真实 URDF + 障碍：Dock「碰撞与规划」起终点 → 规划 → 确认插入 → **回放无 Pmid 碰撞**。
- 若仍报「规划路径未通过画面碰撞校验」：对照 `pose=fk-bind` 与画面 `applyPerLinkRobotBasePlacement`，确认绑定表在规划前刷新。

## 功能扩展

- 外轴与 `TeachIk` 联动（当前仅臂关节 + 数值 IK）。
- coal/FCL 加速碰撞（大 mesh 场景）。
- 规划结果落程序 / PathPlan Apply（本期明确不做）。
- 笛卡尔直线避障 / Pilz LIN。
- SRDF 精细自碰排除（当前 ACM + URDF 邻接）。

## OMPL SDK（若需本地重建）

1. 按 [`bin/SDK/ompl/README.md`](../../../bin/SDK/ompl/README.md) 编译 Boost + OMPL 至 `bin/SDK/ompl/install/`。
2. `RobotPathPlanning.vcxproj` 定义 `CLOUDSIM_HAS_OMPL`，Debug/Release 均可链 `ompl.lib`。
