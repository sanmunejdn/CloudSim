# ACCEPTANCE_IO信号与流程

| 项 | 状态 |
|----|------|
| 信号 Tab 增删 + 工程恢复 | 实现完成，待手工点验 |
| SET_DO / IF(Io) 信号名 | 实现完成 |
| DI 强制 | 实现完成 |
| DEVICE_AXIS | 实现完成 |
| Debug+Release 编译 | 见 FINAL |

手工点验：

1. 信号页添加 DI `Start`、DO `Gripper`
2. SET_DO 选 `Gripper`，IF 条件选 `Start`，强制 DI 后分支应变化
3. 插入 DeviceAxis，指定自定义设备 id/轴/目标 q，Run 后位姿变化
4. 保存工程重开，信号表恢复
