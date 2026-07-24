# TODO — 机器人通讯

## 必须配置

1. **HSL 授权**：设置环境变量 `HSL_AUTH_CODE`，或在 Bridge 输出目录放置 `hsl.auth`（勿提交 git）
2. **启动 Bridge**：运行 `bin\x64d\RobotComm\RobotCommBridge.exe`（或 Release 对应路径）后再在 UI 连接
3. **机侧前提**
   - ABB：RWS 开启，默认用户 `Default User` / `robotics`，端口常 80
   - Fanuc：以太网接口服务；位姿/关节寄存器默认 `D751` / `D777`（可按现场改 Bridge 请求字段）
   - KUKA：机侧 TCP 程序（HSL `KukaTcpNet`），变量 `$AXIS_ACT` / `$POS_ACT`

## 建议后续

- 将 Bridge 随 CloudSim 启动（可选子进程）
- ABB `GetRobotTarget` 四元数 → ZYX 欧拉完整转换
- `IRobotIoSink` 经 Bridge 读写 DI/DO
- 手眼标定改用统一 `get_feedback` 替代相机插件 `GET_POSE`
- 联调：HSL Demo 自带 `FormAbbServer` / `FormFanucRobotServer`

## 重新获取 HSL DLL

```powershell
powershell -File scripts/fetch_hslcommunication.ps1 -Version 12.9.1
```
