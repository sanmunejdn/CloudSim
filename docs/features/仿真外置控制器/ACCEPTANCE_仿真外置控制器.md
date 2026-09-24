# ACCEPTANCE — 仿真外置控制器 MVP

## 验收结论

| # | 标准 | 结果 |
|---|------|------|
| 1 | CONSENSUS 协议冻结；`docs/features/README.md` 可到达 | 通过 |
| 2 | Python 示例连 `127.0.0.1:19620`，`step` 驱动关节（ExternalController） | 通过（mock 冒烟）；桌面需勾选「外置控制器」后跑 `example_sine_joints.py` |
| 3 | 控制器进程崩溃不拖垮 Host | 通过（TCP 断连 `closeClient`，Host 继续 listen） |
| 4 | Debug+Release 生成；无私改 OutDir | 通过：`CloudSimControllerSDK` / `RobotScene` / `CloudSimHost` / `RobotWidget` → `bin\x64d` / `bin\x64` |
| 5 | 列出手工步骤与未做项 | 见下 |

## 手工复现（桌面）

1. 启动 Debug：`bin\x64d\CloudSim.exe`（或 Release `bin\x64`）。
2. 加载带机器人的场景/工程。
3. 仿真指令页勾选 **外置控制器**（或打开 **设置…** 对话框勾选「启用 Host 监听」）；日志应出现 `127.0.0.1:19620`。
4. 可选：在设置对话框选择 Python、查看/编辑源码，点 **运行控制器**（须已 listen）；或另开终端：

```text
cd CloudSim\resource\Python\ControllerPython
python example_sine_joints.py
```

（或已拷贝的 `bin\x64d\resource\Python\ControllerPython\`）

5. 观察关节角随正弦目标变化；Ctrl+C 后发 `GOODBYE`。停发 STEP 时关节应保持末姿态。
6. 取消勾选；再点程序 Run 应恢复原指令回放。勾选外置时点 Run 应提示先关闭外置。

### P0 语义（补充验收）

- `STEP_REPLY.actualJointRad` 与 Host 文档关节真源一致（非仅回显命令）。
- `dtMs` 非 16 倍数时 Host 可警告，帧仍应用。
- 控制器断连后姿态保持，Host 仍 listen。

## 手工复现（离线 mock，无 Host）

```text
python mock_host_server.py
python example_sine_joints.py
```

应打印 `jointCount=6` 与递增 `simTimeMs`。

## Headless / Gateway

- 源码：`HeadlessRobotPlaybackBridge::{start,stop}ExternalController` + timer 分支；Gateway  
  `POST /api/robot/external-controller/start|stop`。
- Headless stub `OsgWidget.h` 与桌面共用头卫 `WIDGET_OSGWIDGET_H`（stub 优先于 include path），Debug+Release 可编过 `CloudSimHostHeadless` / `CloudSimWebGateway`。
- 端到端：先 HTTP start，再 Python 连 `19620` STEP。

## 未做（明确超出 MVP）

- 本机命名管道 / 共享内存 IPC
- ROS bridge
- Webots `<extern>` 双通道
- Sim/Real 同一套高级 API（真机仍走 RobotComm `19610`）
- 二进制大图通道；控制器进程 Launcher

## 与 RobotComm 边界

| | 外置控制器 | RobotComm |
|--|------------|-----------|
| 端口 | `19620` | `19610` |
| 用途 | 驱动**仿真**关节 | 拉**真机**状态 / 桥 |
| SDK | `CloudSimControllerSDK` | `RobotCommSDK` |
| 权威文档 | [CONSENSUS](CONSENSUS_仿真外置控制器.md) | `src/Plugins/RobotCommSDK/DEVELOPER_GUIDE.md` |

## 构建命令（本波验证过）

```text
msbuild src\Plugins\CloudSimControllerSDK\CloudSimControllerSDK.vcxproj /p:Configuration=Debug /p:Platform=x64
msbuild src\Plugins\CloudSimControllerSDK\CloudSimControllerSDK.vcxproj /p:Configuration=Release /p:Platform=x64
msbuild src\Robot\RobotScene\RobotScene.vcxproj /p:Configuration=Debug /p:Platform=x64
msbuild src\Robot\RobotScene\RobotScene.vcxproj /p:Configuration=Release /p:Platform=x64
msbuild src\Host\CloudSimHost\CloudSimHost.vcxproj /p:Configuration=Debug /p:Platform=x64
msbuild src\Host\CloudSimHost\CloudSimHost.vcxproj /p:Configuration=Release /p:Platform=x64
msbuild src\UI\RobotWidget\RobotWidget.vcxproj /p:Configuration=Debug /p:Platform=x64
msbuild src\UI\RobotWidget\RobotWidget.vcxproj /p:Configuration=Release /p:Platform=x64
```
