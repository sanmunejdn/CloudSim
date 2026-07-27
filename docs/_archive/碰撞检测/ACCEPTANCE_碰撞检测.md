# ACCEPTANCE — 碰撞检测

| 验收项 | 结果 |
|--------|------|
| CollisionAlgorithm.dll 可编译 | 通过（Debug x64） |
| RobotWidget/Widget 链接通过 | 通过 |
| 仿真 Dock 有「碰撞检测」页，默认关 | 实现 |
| 开启后 plan 抽样碰撞 | 实现（planMotionOnHost + taught CSV） |
| 几何来自后端 Mesh/B-rep | 实现 |
| 邻接连杆 ACM exclude | 实现（URDF child→parent） |
| project.json `robotCollision` | 实现 |
| 无 coal SDK 仍可运行 | 内置 AABB+三角 |

## 手工建议

1. 关开关：障碍物场景下 LINE 仍可规划  
2. 开开关：障碍穿过 → 规划失败摘要含 backendId  
3. 保存/打开工程后开关状态保持  
