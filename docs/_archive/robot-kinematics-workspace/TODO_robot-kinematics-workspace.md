# TODO — Robot 运动学演进

- 可选：在 Host/调试入口挂 `UrdfRobotLoader::runSelfTest` 便于一键自检
- 可选：有 Pinocchio 环境时用 `tools/pinocchio_oracle/generate_goldens.py` 为真实机型补 golden
- 联立外轴路径仍有局部 `std::vector<double> J` 装配；若 profiler 显示热，可再并入 Workspace
- **已修**：`UrdfNumericalIk` 姿态误差曾误用 `target*inv(cur)`，已改回 `inv(cur)*target`，并恢复 TeachIk 同款 Gauss-Jordan
- 拖动：请用 Debug/Release 各拖一遍确认姿态跟随恢复
