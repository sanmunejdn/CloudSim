# DESIGN — 机器人程序品牌导出

## 架构

```mermaid
flowchart LR
  UI[BrandExportDialog]
  Ctrl[RobotSimulationController]
  Can[Canonical temp JSON]
  Py[PythonScriptCaller]
  Scr[resource ExportPython]
  Out[User program file]
  UI --> Ctrl --> Can --> Py --> Scr --> Out
```

## 组件

| 组件 | 职责 |
|------|------|
| `BrandProgramExportDialog` | 选品牌 |
| `onSimulationExportRequested` | 编排：路径、Canonical、pybind |
| `PythonScriptCaller` | 嵌入解释器 + CallPython |
| `canonical_v1.py` | 校验/遍历 Canonical |
| `*Export.py` | 品牌代码生成 |

## 异常

- 无 URDF / 无运动指令：现有 warning
- 用户取消路径：中止
- 脚本缺失 / Python 初始化失败 / ExportScript 返回 false：RunInfo warning
