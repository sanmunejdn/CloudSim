# TODO — 网页端坐标系对齐

1. **手工点验**：本机启动 `CloudSimWeb.exe`，导入 URDF 后走一遍坐标系 Tab（编辑/捕获/显示/保存重开）。
2. **可选**：桌面 `MainWindowRobotHost::capture*` 改为调用 `RobotCoordinateFrameOps`，去掉重复实现。
3. **可选**：Gateway 侧将 `RobotCoordinate` 调用完全收进 Host JSON 门面，从而 `CloudSimWeb` 可不链 `RobotScene.lib`。
4. 外轴页 / Run Executor / Vite React 壳：仍不在本任务范围（见总 TODO）。
