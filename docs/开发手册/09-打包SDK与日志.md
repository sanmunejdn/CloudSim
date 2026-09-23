# 09 — 打包、SDK 与日志

## 安装打包

权威说明：[Setup/packaging/README.md](../../../../Setup/packaging/README.md)。

| 产品 | 入口 | 安装脚本 |
|------|------|----------|
| Desktop | `CloudSim.exe` | `CloudSim.iss` → `dist\Desktop` |
| Web | `CloudSimWeb.exe` + `web\` | `CloudSimWeb.iss` → `dist\Web` |

公共逻辑：`packaging-common.ps1`（读 `config.ini`、`Find-ISCC`）。流水线：

```powershell
cd Setup/packaging
.\check-environment.ps1
.\build-installer.ps1 -Product Desktop -Clean
.\build-installer.ps1 -Product Web -Clean
```

| 要点 | Desktop | Web |
|------|---------|-----|
| 宿主 | `CloudSimHost.dll` | `CloudSimHostHeadless.dll` |
| 前端 | — | **须** `bin\x64\web\assets\*.js`（拒绝仅 fallback） |
| 插件 / AI runtime | 可收集 | 不收集 |

配置：`config.ini` 的 `QtRoot` / `InnoSetupPath`。AI 运行时（Ollama 等）为 Desktop 可选勾选，细节见 packaging README。

## 运行时目录与 SDK

| 路径 | 用途 |
|------|------|
| `bin\x64d\` / `bin\x64\` | 产品运行目录（exe、DLL、`plugins\`、`web\`、`resource\`） |
| `bin\SDK\`（若存在） | 对外头文件 / 导入库；是否提交以仓库现状为准 |

判定是否保留某目录：vcxproj 或运行时明确引用 → 保留；仅重复 zip / 安装缓存 → 可候选清理（删前再确认）。

## RunLogger

| 入口 | 说明 |
|------|------|
| [RunLogger DEVELOPER_GUIDE](../../src/Infra/RunLogger/DEVELOPER_GUIDE.md) | 实现真源 |

要点：`spdlog` 后端；`RunLogger::info/warn/error`；与运行信息页桥接；x64 为独立 `RunLogger.dll`。

---

← [08 网页端](08-网页端.md) · [手册目录](README.md)
