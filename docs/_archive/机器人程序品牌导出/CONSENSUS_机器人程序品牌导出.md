# CONSENSUS — 机器人程序品牌导出

## 需求与验收

1. Export 可选 ABB / AIR / FANUC / INOVANCE / LineHeating / ROKAE
2. 最终品牌程序路径由 `QFileDialog` 指定，生成文件非空
3. 转换脚本仅从 `resource/Python/ExportPython/` 加载
4. C++ 先写 Canonical（默认 temp），再 pybind 调 `ExportScript`
5. 失败有中文提示，不崩溃

## 技术方案

```
选品牌 → 选输出路径 → IK+Canonical → temp JSON
  → PythonScriptCaller::CallPython(BrandExport.py, ExportScript, {OutPutPath, CanonicalPath})
  → 品牌程序文件
```

| 品牌 | 脚本 | 扩展名 |
|------|------|--------|
| ABB | ABBExport.py | .MOD |
| AIR | AIRExport.py | .arl |
| FANUC | FANUCExport.py | .LS |
| INOVANCE | INOVANCEExport.py | .pro |
| LineHeating | LineHeatingExport.py | .LS |
| ROKAE | ROKAEExport.py | .mod |

入口：`ExportScript(OutPutPath, CanonicalPath, comment="")` → `"true"`/`"false"`；兼 `run(params)`。

## 约束

- Python Home：`bin/SDK/python311`
- Canonical：`format=cloudsim.program_export`，`schemaVersion=1`；位姿 `eulerDeg`（度）
- 导入预留，本次不实现
