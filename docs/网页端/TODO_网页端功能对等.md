# TODO — 网页端功能对等

1. **P3 算法加厚**：`/api/geometry/op`、`/api/pointcloud/op` 当前为作业面 + SSE；需把 PointCloudAlgorithm / GeometryAlgorithm / Vcg / Collision 具体算子参数与结果写回 `objects[]`。  
2. **P2 回放编排**：`/run`/`/stop` 与万级轨迹二进制帧通道（对齐桌面批绘）。  
3. **P5 AI**：`/api/ai/chat` 转发现有 AiLlmClient / 分域 Handler（读 `ai_config.json`）。  
4. **H2 链接期去 OSG**：Host 拆可选渲染后，Web 进程可不链 `OsgWidgetCore` DLL（现已运行期无 OsgWidget）。在拆分完成前 **勿** 从 `CloudSimWeb.sln` 移除 BackendVisual/OsgWidgetCore，否则 VS 会把 ProjectReference 解析到 Win32/`bin\x86d`。  
5. **前端 npm 构建**：本机有 Node 时对 `web/cloudsim-web-ui` 跑 `npm run build:debug|release` 替换 fallback。  
