# TODO_网页端机器人页完整同步

## 审查修复（2026-08-31）

已修：`RobotProgramStore::ensureRobotBackendId`；PUT/导入登记实例；碰撞 confirm 绑定 plan 的 sceneRoot；switch 先校验再切；playback 按 scene 取程序。

## 待办（可选后续）

1. **通讯 Host API**  
   缺 `/api/robot/comm/*` 与 Host 侧 Bridge 轮询/镜像定时器；当前页签仅说明降级路径。

2. **碰撞 OMPL / OSG 预览**  
   网页规划为 JointLerp+FK；桌面黄线预览与 OMPL 避障未移植。

3. **手测清单**（需本机开 `CloudSimWeb`）  
   - 导入机器人 → 新建/改名/删程序 → Ctrl+Z/Y  
   - 建组/解散/改名  
   - IK 链式 vs 当前后 plan/Run  
   - 碰撞白名单 + 起终点规划 → 确认插入  
   - 外轴读写后规划是否带入上下文

## 配置

无额外 `.env`；通讯需自行启动 `tools/RobotCommBridge`（仅桌面完整可用）。
