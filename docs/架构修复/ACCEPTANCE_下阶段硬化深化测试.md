# ACCEPTANCE：下阶段硬化深化测试

总单 BugTracker **#11**。

## 验收清单

| ID | 项 | 状态 | 证据 |
|----|----|------|------|
| N1 | `check_host_export_surface.py` 绿；挂入 make-source-check；故意缺基线可失败 | ✅ | Debug/Release DLL 基线校验通过；脚本已挂 `host-export-surface` |
| N1 | WHOLEARCHIVE 仍在 Host/Headless | ✅ | 未改链接选项；设计 §8 区分纳入 vs 表面 |
| N2 | PluginGeometry/PointCloud HostImpl + DocumentPointCloudOps 以分面为主 | ✅ | 目标文件 `widgetOsgFromPage`/`IOsgWidgetView*` = 0 |
| N3 | import/project/visual/follow/adapters/io 主路径分面 | ✅ | 范围目录 `osgWidgetFrom`/`IOsgWidgetView*` = 0 |
| N4 | WidgetDocumentAccess 分面 helper；SceneWiring → PickObserve + Signals | ✅ | DocumentPage chrome 已用 toolbar/sceneOps |
| N5 | JobSystem lifecycle SelfTest 可自动化 | ✅ | `JobSystemSelfTest.exe` + `run_selftest.ps1` |
| N5 | LIFECYCLE_CONTRACT 纠正虚标；Three.js 默认 CI 不阻塞 | ✅ | 契约表已更新 |
| 双配置 | Host/Headless/Widget Debug\|x64 + Release\|x64 | ✅ | 见构建日志 |
| 门禁 | make-source-check 全绿 | ✅ | 2026-09-29 全步骤 OK（含 host-export-surface） |

## 回归用例

1. `python scripts/check_host_export_surface.py` / `--config release`
2. `powershell -File scripts/make-source-check.ps1`
3. `JobSystemSelfTest.exe`（或 `run_selftest.ps1 -Preset gate`）
4. 冒烟：导入几何 / 点云预览 / 关窗不挂死

## 明确未做

- 删 WHOLEARCHIVE、SelectionOperation owner、RobotWidget 分面、Playwright
