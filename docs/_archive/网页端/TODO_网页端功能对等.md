# TODO — 网页端功能对等

1. **P3 算法加厚**：`/api/geometry/op`、`/api/pointcloud/op` 当前为作业面 + SSE；需把 PointCloudAlgorithm / GeometryAlgorithm / Vcg / Collision 具体算子参数与结果写回 `objects[]`。  
2. **P2 回放编排**：`/run`/`/stop` 与万级轨迹二进制帧通道（对齐桌面批绘 / `RobotProgramExecutor`）。  
3. **P5 AI**：`/api/ai/chat` 转发现有 AiLlmClient / 分域 Handler（读 `ai_config.json`）。  
4. **H2 链接期去 OSG**：Host 拆可选渲染后，Web 进程可不链 `OsgWidgetCore` DLL（现已运行期无 OsgWidget）。在拆分完成前 **勿** 从 `CloudSimWeb.sln` 移除 BackendVisual/OsgWidgetCore，否则 VS 会把 ProjectReference 解析到 Win32/`bin\x86d`。  
5. **前端 npm 构建**：本机有 Node 时对 `web/cloudsim-web-ui` 跑 `npm run build:debug|release` 替换 fallback。  
6. ~~设备库（URDF）+ 轴控制页~~：**已完成**（`HeadlessRobotContext`、`/api/devices/catalog`、`/api/robot/instances|joints` GET、左坞瓦片、右坞轴控制滑条）。  
7. **PLC / 工业相机**：仍为 stub，与设备库无关。  
8. ~~指令程序编辑器（基础）~~：**已完成**（示教 PTP/LINE/ARC、GET/PUT programs、选中规划预览、属性 GET/PATCH；Run/导出仍 stub）。  
9. **轴控制增强（可选）**：外部轴滑条、可达域、可行轴配置 UI。  
10. ~~PathPlan / 轨迹管线~~：**已完成**（`HeadlessTrajectorySession`、拾取/离散/Mesh、管线预览应用、模板与 undo；细节见 `docs/网页端轨迹对等/`）。
