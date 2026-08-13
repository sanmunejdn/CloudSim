# ALIGNMENT — 自定义设备

## 原始需求

独立自定义设备：导入 STEP/网格等几何，组合成设备；定义单轴平移或旋转（方向/中心/范围）；轴控页可切换设备拖动预览。

## 项目理解

- 设备页现仅扫 URDF 包；机器人外轴已有 Translate/Rotate + axis/origin/limits
- Data 无机构设备聚合根；父子 + FollowAttachment 可做整机跟随
- 空间契约：`worldMatrix` 为位姿真源

## 边界

| 在范围内 | 不在范围内 |
|----------|------------|
| `CustomDeviceBackendData` 聚合根 + 子件 | 与机器人联立 IK / 程序外轴 |
| 单轴 MVP，ConfigSet 预留多轴 | 多 link 编辑器、导出 URDF 包 |
| 轴控页切换机器人/设备 | Web DevicesPanel 对等 |
| 工程 objects 持久化运动字段 | 3D 拾取旋转中心 |

## 疑问与决策

| 项 | 决策 |
|----|------|
| 新 className | 是，`CustomDeviceBackendData` |
| 运动存储 | 设备派生 JSON，非旁路 sidecar |
| Data 依赖 RobotScene | 否；Data 自含轴配置，FK 在 RobotScene 转调 `RobotExternal` |
| 整机 gizmo | apply 前若世界矩阵与 `W0*T(q)` 漂移则 unbake W0 |
