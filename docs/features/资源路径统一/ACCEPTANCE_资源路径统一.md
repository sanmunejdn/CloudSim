# ACCEPTANCE — 资源路径统一

## 验收对照

| # | 标准 | 状态 |
|---|------|------|
| 1 | 无 `CloudSim\resource\` | 通过 |
| 2 | Controller 源在 `RobotWidget\resource\` | 通过 |
| 3 | OutDir 含 Controller / controllers / models（构建后） | 通过（x64d + x64） |
| 4 | `bin\resource` 仅 `models` | 通过 |
| 5 | 对话框回退无旧路径 | 通过（`ExternalControllerDialog.cpp`） |
| 6 | Debug+Release 编译通过 | 通过（RobotWidget + CloudSim） |

## 开发口诀（交付说明）

- 改脚本/JSON → `CloudSim\src\...\resource\`
- 改模型 → `bin\resource\models\`
- 跑程序 → `bin\x64(d)\resource\`
