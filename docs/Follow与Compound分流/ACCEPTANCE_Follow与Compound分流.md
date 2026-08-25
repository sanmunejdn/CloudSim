# ACCEPTANCE — Follow 与 Compound 分流

| # | 标准 | 状态 |
|---|------|------|
| 1 | attach / edges 不再写入 hierarchyDriven Follow | 代码已改 |
| 2 | 加载/求解剥离 hierarchyDriven；显式 Follow 保留 | 代码已改 |
| 3 | 连杆 FK `applyToSink` 用共用 compound API，跳过自身已启用 Follow 的子 | 代码已改 |
| 4 | Follow 后未挂载设备 `W0+applyQ`；其它根 compound；再解跟 target 的 Follow | 代码已改 |
| 5 | 挂载 refresh 后仍有一轮跟连杆子件的 Follow | 代码已改 |
| 6 | 属性/gizmo 提交父位姿时 compound（非 CustomDevice） | 代码已改 |
| 7 | flushCustomDeviceLinkGeometryVisual 不调 Follow | 保持 |
| 8 | Debug\|x64 + Release\|x64 相关工程编译通过 | 已通过（Data / RobotScene / CloudSimHost / Headless / Widget） |
