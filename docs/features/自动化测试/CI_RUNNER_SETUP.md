# CI Runner 配置（self-hosted）

GitHub Actions **不能**用公共 Windows runner：依赖本机 `../bin/SDK`（OCC/CGAL/OSG/Qt）。

## 标签

注册 runner 时加标签：`self-hosted`, `Windows`, `cloudsim`（与 `.github/workflows/ci.yml` 一致）。

## 机器要求

| 项 | 说明 |
|----|------|
| OS | Windows 10/11 x64 |
| VS | VS 2019/2022 + MSVC v142 + MSBuild |
| Qt | 5.14.2_msvc2017_64（与工程 QtInstall 一致） |
| Python | 3.10+（`python` 在 PATH） |
| Node | 可选；网页 PostBuild 需要 npm（或已有 `bin/*/web`） |
| SDK | 仓库旁 `bin/SDK` 完整（与日常开发机相同） |
| 磁盘 | 构建产物 + artifacts 建议 ≥ 20GB 空闲 |

## 注册步骤（摘要）

1. GitHub → Settings → Actions → Runners → New self-hosted runner
2. 按提示下载并 `.\config.cmd --labels cloudsim`
3. `.\run.cmd` 常驻（或注册为 Windows 服务）
4. 工作目录指向本仓库 clone（根目录含 `CloudSim.sln`）

## 本地等价命令

```powershell
cd <repo>   # CloudSim/
.\scripts\run_checks.ps1
.\scripts\run_soak.ps1 -Rounds 200
```

## 失败产物

`artifacts/checks/`、`artifacts/soak/`、`bin/x64d/crash/`；可用：

```powershell
python scripts/collect_failure_bundle.py --bin-dir ..\bin\x64d
```
