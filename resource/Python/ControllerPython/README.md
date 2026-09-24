# CloudSim External Controller — Python 最小客户端

协议：TCP `127.0.0.1:19620`，一行一条 UTF-8 JSON（`protocolVer=1`）。字段以仓库内 CONSENSUS 为准，勿私改。

## 文件

| 文件 | 说明 |
|------|------|
| `cloudsim_controller.py` | `RobotController`：`connect` / `hello` / `step` / `goodbye` |
| `example_sine_joints.py` | 正弦关节目标示例 |
| `mock_host_server.py` | 无 CloudSim 时的离线 mock Host |

## 相对 Host 运行

1. CloudSim Host 开启 **ExternalController**，listen `127.0.0.1:19620`
2. 进入本目录（或已拷贝到运行目录的副本）：

```text
cd CloudSim\resource\Python\ControllerPython
python example_sine_joints.py
```

3. Ctrl+C 结束；客户端会发 `GOODBYE` 并断开

### 拷贝到 bin（如需要）

Host 若从 `OutDir` 下 `resource` 加载资源，请把本目录同步到：

- Debug：`bin\x64d\resource\Python\ControllerPython\`
- Release：`bin\x64\resource\Python\ControllerPython\`

源码树路径为 `CloudSim\resource\Python\ControllerPython\`；**不要**只拷到其它自定义目录。

## 离线冒烟（mock，无需 Host）

终端 1：

```text
cd CloudSim\resource\Python\ControllerPython
python mock_host_server.py
```

终端 2：

```text
cd CloudSim\resource\Python\ControllerPython
python example_sine_joints.py
```

应看到 `jointCount=6` 与不断打印的 `simTimeMs` / `actual`。Ctrl+C 退出 example；mock 可继续接下一客户端。

可选：用 `nc` / 其它工具向 `127.0.0.1:19620` 发一行 JSON 自测（仍须遵守冻结字段）。

## 依赖

仅 Python 标准库（`socket`、`json`）。建议 Python 3.7+。
