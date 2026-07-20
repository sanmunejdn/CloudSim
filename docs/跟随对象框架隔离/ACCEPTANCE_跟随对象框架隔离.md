# ACCEPTANCE — 跟随对象框架隔离

## 子任务完成

| ID | 状态 | 说明 |
|----|------|------|
| T1 | 完成 | `isKinematicsOwnedBackend` / `stripKinematicsOwnedFollowAttachments` |
| T2 | 完成 | hierarchy / edges 跳过 URDF child |
| T3 | 完成 | 求解入口 strip；sync 跳过 URDF；属性绑定取消 forced |
| T4 | 完成 | DEVELOPER_GUIDE + 本目录文档 |

## 编译

- `CloudSimHost` ClCompile（`BuildProjectReferences=false`）通过。
- 全量 Link 仍可能因环境缺 `GeometryEngine.lib` 失败（与本改无关）。

## 手工验收（待你本地跑）

1. 打开含机器人场景；确认装配完整。
2. 选中工件，设跟随目标为法兰/末端：工件位置不跳变。
3. 拖动关节 / 跑仿真：工件跟随，机器人不散架。
4. （可选）打开旧工程后看连杆是否仍带 Follow；应被 strip。
