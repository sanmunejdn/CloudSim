# ACCEPTANCE — 机器人程序品牌导出

## 检查项

| # | 标准 | 结果 |
|---|------|------|
| 1 | Export 可选程序（当前机器人 catalog）与 6 品牌；最终路径由对话框指定 | 已实现（`BrandProgramExportDialog` 程序+品牌 + `QFileDialog`） |
| 2 | 脚本仅从 `resource/Python/ExportPython/` 加载 | 已实现；PostBuild 复制到 `bin/*/resource/...` |
| 3 | Canonical 为中间产物（temp） | `QTemporaryFile` |
| 4 | pybind 嵌入调用 `ExportScript` | `PythonScriptCaller` |
| 5 | 脚本直接读 Canonical v1 | `canonical_v1.py` + 6 品牌脚本 |
| 6 | RobotWidget Debug 链接通过 | 2026-07-20 `CloudSim.sln /t:RobotWidget` 成功 |
| 7 | 离线脚本冒烟（fixture → ABB/FANUC/INOVANCE） | `ExportScript` 返回 `true`，生成非空文件 |
| 8 | 万级点不卡在全量 IK | 品牌导出跳过 per-point IK；紧凑 JSON；Python 流式写 |

## 未测（需 GUI）

- 完整 UI 点击 Export → 选 ABB → 保存 .MOD 的人工验收
- 无 Python SDK / 脚本缺失时的中文 warning 文案肉眼确认
- **万级路点**端到端耗时对比（优化前后）

## 构建产物

- `bin/x64d/RobotWidget.dll`（含品牌导出）
- `bin/x64d/resource/Python/ExportPython/*.py`
- `bin/x64d/python311.dll`（随 PostBuild 复制）
