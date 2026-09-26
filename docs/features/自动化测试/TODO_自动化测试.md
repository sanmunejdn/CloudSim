# TODO — 自动化测试

## 需本机/运维完成

1. **注册 self-hosted runner**（标签 `cloudsim`），按 [CI_RUNNER_SETUP.md](CI_RUNNER_SETUP.md) 配置；首次 CI 跑绿后归档截图。
2. **准备 `bin/*/resource/models`** 后，机器人 API 用例不再 skip，可补强 tcp-ik / 回放断言。
3. **Dump 验证**：`$env:CLOUDSIM_WEB_CRASH_ON_START='1'; ..\bin\x64d\CloudSimWeb.exe --port=8799`，确认 `crash\*.dmp` 落盘后删环境变量。
4. **soak 200 轮**：`.\scripts\run_soak.ps1 -Rounds 200`（首次建议先 `-Rounds 20` 冒烟）。
5. **Release CloudSim.exe**：按需编 Release 桌面端（Debug 已含 CrashDump）。

## 已知技术债

| 项 | 说明 |
|----|------|
| VcgAlgorithms Debug assert | `vcg/complex/allocate.h` 退化三角；gate 已跳过，full+Debug 仍会 abort |
| GeometryAlgorithm full | 全量自检可达数分钟以上，勿放 push 门禁 |
| Qt 路径硬编码兜底 | `D:\Qt\Qt5.14.2\...`；优先设 `QTDIR` |

## 操作指引（常用）

```powershell
cd CloudSim
.\scripts\run_selftest.ps1 -Preset gate
.\scripts\run_api_tests.ps1 -Configuration Debug
.\scripts\run_checks.ps1          # 双配置编译+自检+API
.\scripts\run_soak.ps1 -Rounds 20
python scripts\collect_failure_bundle.py --bin-dir ..\bin\x64d
```
