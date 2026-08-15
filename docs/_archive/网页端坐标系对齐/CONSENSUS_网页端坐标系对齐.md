# CONSENSUS — 网页端坐标系对齐

## 需求

网页「坐标系」Tab 对等桌面 Frames：工具/用户列表、法兰 link、位姿/欧拉、增删复制、设为当前、TCP 捕获、重置工具系、全局/单项显示、Three.js 叠加轴；工程保存写入 `robotKinematicsInstances[].coordinateFrames`。

## 验收标准

1. GET/PUT `/api/robot/frames` 读写完整 frame set + link 名列表
2. POST capture-tool / capture-user / reset-tool 语义对齐桌面
3. GET overlays 返回可见系世界位姿；前端画出 RGB 轴
4. 面板可完成 CRUD / 激活 / 显示 / 编辑；debounce PUT
5. 保存工程后再打开，坐标系一致（桌面↔网页互开不丢）
6. Active/几何变更后示教/规划仍用新工具（context 同步）
7. CloudSimHost → Gateway → CloudSimWeb Debug|x64 与 Release|x64 均通过

## 约束

- 桌面 Widget 零强制大改；Host 抽出共享 helper
- 空间契约见 `docs/spatial_contract_world_pose.md`
- 静态资源须进 `bin\x64d\web` / `bin\x64\web`

## 技术方案要点

- Host：`RobotCoordinateFrameOps` + Headless 封装 + `mergeRobotKinematicsIntoProjectRoot`
- Gateway：frames / capture / reset / overlays + SSE `RobotCoordinateFramesChanged`
- Frontend：替换 `#robotFrame` 占位
