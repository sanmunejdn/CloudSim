# FINAL — 跟随对象框架隔离

## 结论

机器人解体的框架根因是：**工程层级 edges 把 URDF 连杆当成 Follow follower**，工件绑定时 **forced 全量求解** 用 Follow 公式改写连杆 `pose`，与 FK 冲突导致散架。

已用硬边界隔离：URDF 对象位姿仅 FK 写；Follow 只服务自由物体。

## 改动摘要

- `DocumentHost`：所有权查询 + strip
- `BackendHierarchyFollow` / `BackendProjectObjectIo`：不绑 URDF
- `BackendFollowSolve`：求解前 strip；属性绑定脏集求解

## 后续

见 `TODO_跟随对象框架隔离.md`。
