# ProcessFlowPlugin 开发指南

## 1. 定位

| 项 | 说明 |
|----|------|
| 解决方案 | `CloudSim.sln` |
| 产物 | `bin/*/plugins/com.cloudsim.processflow/` |
| 职责 | 工艺流程图编辑 + DES 离散事件仿真 |

## 2. 边界

- 进入模式后中央三维被流程页替换；仿真与 AI 坞叠放
- 细节与节点类型见 [README.md](README.md)
- 专题入口：[工艺流程](../../features/工艺流程/README.md)

## 3. 验证

Debug|x64 + Release|x64 编本工程。
