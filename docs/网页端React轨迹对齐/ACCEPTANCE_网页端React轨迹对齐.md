# ACCEPTANCE — 网页端 React 轨迹对齐

| # | 验收项 | 结果 |
|---|--------|------|
| 1 | ToWorkpiece 外部 TCP 列出插入坐标系 | **通过**（`sceneFrames` + radio） |
| 2 | 选中帧写入 `externalTcpBackendId`，隐藏手动六自由度 | **通过**（既有 `isOpFieldVisible`） |
| 3 | 拾取完成后高亮清除 | **通过**（`SceneViewport`） |
| 4 | 取消拾取 / 指针离开清除高亮 | **通过** |
| 5 | 应用成功后退出编辑态并清空特征/预览 | **通过**（`exitEditAfterCommit`） |
| 6 | 生成成功后同样复位并 `goCmd` | **通过** |
| 7 | 绑定 PathPlan / 取消修改清空本地态 | **通过** |
| 8 | `npm run build:debug` + `build:release` | **通过** → `bin\x64d\web` / `bin\x64\web` |
| 9 | 开发文档更新 | **通过**（本目录 + `DEVELOPER_GUIDE.md` + 索引） |

## 手工点验（建议）

1. 插入坐标系 → 轨迹编辑 → ToWorkpieceInHand → 选该帧 → 预览/应用  
2. 开始修改 → 拾取面 → 确认高亮消失 → 离散见 Raw  
3. 应用 → 指令树有 LINE；轨迹生成显示「未开始修改」、特征表空；再进编辑页流水线空
