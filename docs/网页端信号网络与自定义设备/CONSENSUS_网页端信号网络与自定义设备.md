# CONSENSUS — 网页端信号网络与自定义设备

## 需求与验收

1. 打开含 `ioSignalNetwork` 的桌面工程：网页可见 Owner、连线；保存后桌面可读
2. DO 写值 → 目标 DI 传播；设备 DI 上升沿可驱动姿态
3. SignalsPanel：Owner 选择 + 信号连接站（React Flow）
4. DevicesPanel：自定义设备运行面 + 组装 Apply
5. 程序 Run：SET_DO / SET_AO / WAIT(IO) 走同一网络 runtime

## 技术方案

- Host：`IoSignalNetwork` + `CustomDeviceHostOps`
- Gateway：`/api/io/network*`、`/api/custom-devices*`
- 工程保存写 `ioSignalNetwork`，删除 `ioSignals`

## 边界

- 闭环机构、面拾取原点：不做
- URDF 导出 ZIP：Host 已有 API，网页 UI 本期可不暴露
