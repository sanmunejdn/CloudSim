# FINAL — 机器人程序品牌导出

## 交付摘要

完善仿真 Export：Canonical v1 为统一中间产物；用户选品牌与最终程序路径；pybind 调用 `resource/Python/ExportPython` 下脚本生成 ABB/AIR/FANUC/INOVANCE/LineHeating/ROKAE 程序。

## 主要变更

| 路径 | 说明 |
|------|------|
| `RobotWidget/resource/Python/ExportPython/` | `canonical_v1.py` + 6 品牌导出脚本 |
| `PythonScriptCaller.*` | 嵌入 python311 + CallPython |
| `BrandProgramExportDialog.*` | 品牌选择 |
| `RobotSimulationController::onSimulationExportRequested` | 编排导出 |
| `RobotWidget.vcxproj` | pybind/python 链接 + 复制脚本与 python DLL |
| `docs/机器人程序品牌导出/` | 6A 文档 |
| DEVELOPER_GUIDE（RobotWidget / RobotScene） | 导出说明更新 |

## 验证

- 脚本单元：fixture → ABB/FANUC/INOVANCE 成功
- 工程：`CloudSim.sln` 构建 `RobotWidget` Debug x64 成功
