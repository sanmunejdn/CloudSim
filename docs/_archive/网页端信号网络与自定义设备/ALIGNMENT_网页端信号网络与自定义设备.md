# ALIGNMENT — 网页端信号网络与自定义设备

## 原始需求

桌面端多 Owner「信号连接站」与自定义设备（运行绑定 → 组装画布）同步到网页端。

## 边界

- 工程侧车统一为 `ioSignalNetwork`；兼容读取旧 `ioSignals`
- 逻辑在 Host/Gateway；React 只编辑展示
- UI 扩展 LeftDock；连接站/组装用模态 overlay

## 需求理解

- 网页曾用扁平 `namedSignalTable`/`ioSignals`，与桌面 `ioSignalNetwork` 互通会丢线
- 自定义设备运行面与组装分两期，组装契约同桌面 `links`/`joints` JSON

## 澄清（已锁定）

- Host 自持 `IoSignalNetwork`（与桌面 JSON 同形），不反向依赖 RobotWidget
- 旧 `/api/io/signals*` 薄封装到主机器人 Owner
