# FINAL — 仿真外置控制器 MVP

## 交付摘要

按计划落地 Webots 风格外置控制器 MVP：localhost TCP `19620`、JSON 行协议、`ControlContext` + `ControllerManager`（RobotScene）、`CloudSimControllerSDK`、Python 最小客户端、桌面 ExternalController 勾选接线、Headless/Gateway 同源 API 源码。

## 验证

- Debug|x64 + Release|x64：`CloudSimControllerSDK`、`RobotScene`、`CloudSimHost`、`RobotWidget` 通过。
- Python mock 冒烟：`HELLO`/`STEP` 循环正常。
- 协议与文档入口：`docs/features/仿真外置控制器/` + `docs/features/README.md`。

## 已知限制

见 [TODO_仿真外置控制器.md](TODO_仿真外置控制器.md)（Headless 全量链接既有 OsgWidget stub 问题；未做 IPC/ROS/真机同一 API）。
