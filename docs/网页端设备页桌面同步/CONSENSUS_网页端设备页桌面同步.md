# CONSENSUS — 网页端设备页桌面同步

## 验收

1. 右栏二级「设备」，其内「机器人 | 自定义设备」
2. 自定义设备 → 设备指令：示教/CRUD 姿态、DI 下拉绑定、durationSec、保存
3. 轴控制可切机器人实例与自定义设备
4. 左栏设备仅目录/组装 +「打开设备指令」
5. `goTrajGen` / `goCmd` 仍切到机器人侧

## 技术约束

- 导航：`dockNavStore`（`ws=devices`、`deviceMode`、`deviceTab`）
- 共享选中设备：`deviceRuntimeStore`
- API 不变：`/api/custom-devices*`、`/api/io/network`
- 上升沿仍由 Host `processCustomDevicePoseRisingEdges` 处理（瞬时 applyQ）
