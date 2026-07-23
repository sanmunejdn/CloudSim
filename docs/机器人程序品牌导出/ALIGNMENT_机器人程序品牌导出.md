# ALIGNMENT — 机器人程序品牌导出

## 原始需求

完善机器人程序导出：统一产物为 Canonical v1 JSON；导出时选择品牌；经 pybind 调用 `resource/Python/ExportPython/` 下脚本，将 Canonical 转为品牌程序；最终程序路径由用户对话框选择。导入不在本次范围。

## 项目理解

- 现有导出：`RobotSimulationController::onSimulationExportRequested` → `RobotCanonicalExport` → `*.cloudsim-program.json`
- 参考：HPL `ExportPython/*Export.py` + `HPLPyCaller`（pybind 嵌入）
- CloudSim SDK：`bin/SDK/python311`、`pybind11`

## 边界

| 纳入 | 不纳入 |
|------|--------|
| Canonical 中间产物 | 品牌程序导入 |
| 6 品牌脚本（直接读 Canonical） | RobotInsFile 适配层 |
| pybind 调用 | QProcess |
| 用户选最终程序路径 | 用户选 `.py` 路径 |

## 歧义（已决策）

- 统一格式：Canonical（非 RobotIns）
- 调用：pybind 嵌入
- 脚本目录：`resource/Python/ExportPython/`
