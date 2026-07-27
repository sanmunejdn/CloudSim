# ALIGNMENT — 工业相机插件（海康一期）

## 原始需求

开发 CloudSim 工业相机插件：支持海康等常用工业相机连接/拍照/取图（2D+3D），与机器人手眼标定（眼在手上/外）；通用 `ICamera` + 品牌派生；结果可追查。

## 项目理解

- CloudSim：Qt 5.14 + OSG + 插件体系；插件仅链 `CloudSimPluginSDK`
- 现无工业相机/OpenCV；有 Eigen、点云导入、`resource/` 运行时资源约定
- 参考形态：PlcCommSDK + PlcCommPlugin

## 边界确认

| 纳入一期 | 排除（二期+） |
|----------|----------------|
| 海康 2D MVS、3D Mv3dRgbd | 梅卡 Mech-Eye、大恒/Basler |
| GigE IP 枚举/手填直连 | 品牌机器人官方 SDK（UR/ABB…） |
| 手动末端位姿 + TCP/JSON 真实位姿源 | ethz 时间同步、无板点云手眼 |
| 多算法 Ensemble 择优 | 在线视觉伺服 |
| 落盘 `resource/industrial_camera/` | 改 Host 核心契约（除非必需） |

## 关键假设

1. 本机可无未装 MVS/Mv3d/OpenCV：工程须可编过；运行时提示缺 SDK；提供模拟相机调试 UI
2. 位姿单位 mm，欧拉 ZYX，对齐 spatial contract
3. OpenCV 未部署时：板检测不可用，但允许外部给定 `T_cam_board`；手眼求解用 Eigen 多算法实现

## 疑问（已关闭）

| 问题 | 决议 |
|------|------|
| MVP 品牌 | 仅海康；梅卡二期 |
| 位姿来源 | 真实机器人接口 + 手动输入 |
| 标定算法 | 多算法并行 + 残差择优 |
| 数据落盘 | `bin/.../resource/industrial_camera/` |
| UI | 双 Tab：相机 / 手眼阶段向导 |
| IP | 一等字段，支持手填直连 |
