# controllers/sine_python

最小控制器目录约定示例。

## 布局

```text
controllers/sine_python/
  README.md          # 本文件
  entry.py           # 入口（可转调 resource 示例）
```

## 运行

1. CloudSim 开启外置控制器监听 `127.0.0.1:19620`
2. `python entry.py`（工作目录为本目录，或由设置对话框「运行入口」拉起）

入口脚本会把 `OutDir/resource/Python/ControllerPython` 加入 `sys.path` 并执行正弦示例。
