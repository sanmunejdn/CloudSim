# CONSENSUS — 网页端 React 轨迹对齐

## 需求描述

React 网页端轨迹生成 / 轨迹编辑与 fallback 行为对等：坐标系可选、高亮可清、应用后 UI 复位。

## 验收标准

1. 插入坐标系后，`ToWorkpieceInHand`「外部 TCP」列表可见该帧，选中写入 `toWorkpiece.externalTcpBackendId`
2. 拾取面/线完成后，半透明高亮消失；取消拾取 / 指针离开同样清除
3. `应用` / `生成` 成功后：`featureEditActive=false`、特征表空、Raw 预览关、流水线本地空、切到指令树
4. `npm run build:debug` 与 `build:release` 均成功，产物在仓库根 `bin\x64*\web`

## 技术方案

| 模块 | 方案 |
|------|------|
| 帧列表 | `docks/robot/sceneFrames.ts` → Host API → 本地 objects → `/api/objects` |
| 帧 UI | `OpParamForm` radio（`.op-frame-picker`） |
| 高亮 | `SceneViewport` clear + `hoverPickSeqRef`；`pickMode` 空时 effect 清组 |
| 复位 | `trajectoryStore.exitEditAfterCommit` + `editUiEpoch`；Edit 面板 `goCmd` |

## 技术约束

- 输出目录以 vcxproj / Vite `OutDir` 为准，不自造路径
- 行为歧义以 fallback `app.js` 为准
- 不改 C++ Apply 协议（本轮纯前端）

## 任务边界

仅 React 壳与文档；Gateway/Host 若已提供 API 则复用，不扩协议。
