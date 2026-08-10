# ALIGNMENT — 网页端 React 轨迹对齐

## 原始需求

将 `cloudsim-web-ui`（Vite + React）轨迹生成 / 轨迹编辑行为与布局对齐 `_archive/public-fallback`，并覆盖近期缺陷：

1. 转换工件（`ToWorkpieceInHand`）可选场景中**插入的坐标系**
2. 特征拾取高亮在完成后须清除
3. **应用 / 生成**后轨迹生成与轨迹编辑页须初始化（退出编辑态）

## 项目上下文

| 项 | 说明 |
|----|------|
| 金标 | `web/cloudsim-web-ui/_archive/public-fallback/` |
| 正式壳 | `web/cloudsim-web-ui/src/` → `bin\x64d\web` / `bin\x64\web` |
| Host API | `CloudSimWeb` Gateway；会话 `HeadlessTrajectorySession` |
| 前序归档 | `docs/_archive/网页端轨迹对等/`、`网页端轨迹UI对齐/`、`网页端轨迹编辑对齐/` |

## 边界

**在范围内**

- React 轨迹生成 / 编辑 / 场景拾取高亮 / Raw 预览
- `trajectoryStore.exitEditAfterCommit` 与面板复位
- ToWorkpiece 外部 TCP 帧列表与 radio UI
- 开发文档（本专题 + `DEVELOPER_GUIDE.md`）

**不在范围内**

- 改桌面 `TrajectoryEditPageWidget` 语义
- 改 Host Apply/Emit 协议字段
- PLC / 工业相机 / Run 万级回放
- 强制废弃 fallback（仅作对照）

## 需求理解

- fallback 在 apply/emit 后调用 `exitTrajEditUiAfterCommit`；React 曾只 `syncSession`，本地特征/预览/流水线残留
- 外部 TCP 帧列表须走 `coordinate-frames` + 场景对象回落，且不用易被裁切的原生 select
- 拾取结束后 `pickMode=null` 时须清 overlay，并丢弃过期 hover

## 疑问澄清（已决）

| 问题 | 决策 |
|------|------|
| 应用后是否保留拾取高亮？ | 否，提交后立即清除 |
| 应用后是否切指令树？ | 是，对齐 fallback `refreshInstructionTreeAfterTrajectory` |
| 帧列表是否含非 FrameBackendData？ | 与 fallback 一致：`FrameBackendData` / `CoordinateFrame` / `Frame` |
