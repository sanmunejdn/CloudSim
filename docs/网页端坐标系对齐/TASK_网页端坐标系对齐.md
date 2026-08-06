# TASK — 网页端坐标系对齐

## T1 文档

- 输出：ALIGNMENT / CONSENSUS / DESIGN / TASK
- 验收：目录齐全、范围无歧义

## T2 Host

- 输入：Headless 关节/URDF/frames
- 输出：`RobotCoordinateFrameOps`（capture/reset/overlays/sync）+ `mergeRobotKinematicsIntoProjectRoot`
- 验收：单元级调用可改 frames；保存根含 `coordinateFrames`

## T3 Gateway

- 输出：frames REST + SSE；save 调 kinematics merge；API 草表更新
- 验收：curl/浏览器可读写

## T4 Frontend

- 输出：`#robotFrame` 面板 + overlays 绘制
- 验收：对齐桌面 Frames 操作清单

## T5 构建与验收

- 输出：Debug+Release 通过；ACCEPTANCE / FINAL / TODO
