# BUGFIX_LOOP — 失败 → 修复闭环

## 硬规则

**每个 bug 修复必须补至少一条回归用例**（优先 `tests/api/`；数值问题补对应 DLL `SelfTest`）。

## 流程

1. 本地或 CI 失败 → 保留 `artifacts/checks/<stamp>/`
2. 打包：`python scripts/collect_failure_bundle.py --bin-dir ..\bin\x64d`
3. 打开 bundle：失败日志、junit、crash dump、soak CSV
4. Agent / 人工：定位根因 → 最小修复 → **补回归用例**
5. `.\scripts\run_checks.ps1` 双配置全绿后才算关闭

## 产物落点

| 类型 | 路径 |
|------|------|
| 门禁日志 | `artifacts/checks/<stamp>/*.log` |
| junit | `artifacts/checks/api-*-junit.xml` |
| soak CSV | `artifacts/soak/*.csv` |
| crash dump | `{exeDir}/crash/*.dmp` |
| bundle zip | `artifacts/failures/bundle_*.zip` |

## Dump 解析

Release/Debug 均 `GenerateDebugInformation=true`。用 VS 或 WinDbg 打开 `.dmp`，加载同配置 PDB（与 exe 同目录）。

## 验证 dump（勿用于生产）

```powershell
$env:CLOUDSIM_WEB_CRASH_ON_START='1'
..\bin\x64d\CloudSimWeb.exe --port=8799
# 应在 crash\ 下生成 .dmp
Remove-Item Env:CLOUDSIM_WEB_CRASH_ON_START
```
