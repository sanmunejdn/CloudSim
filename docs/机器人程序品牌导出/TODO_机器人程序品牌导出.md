# TODO — 机器人程序品牌导出

## 待办

1. **GUI 人工验收**：在 CloudSim 中导入 URDF、添加 PTP/LINE，点 Export，选 ABB，确认 `.MOD` 可打开。
2. **Release 构建**：确认 `bin/x64/resource/Python/ExportPython/` 与 `python311.dll` 已复制。
3. **导入（后续任务）**：品牌程序 / Canonical 回编辑器未做。

## 运行依赖

| 项 | 位置 |
|----|------|
| Python 运行时 | `bin/SDK/python311`（相对 exe 为 `../SDK/python311`） |
| python311.dll | 需在 exe 同目录（PostBuild 已拷） |
| 品牌脚本 | `resource/Python/ExportPython/*.py` |

若提示「未找到 SDK python311」：检查 exe 是否在 `bin/x64` 或 `bin/x64d`，且 `bin/SDK/python311` 存在。
