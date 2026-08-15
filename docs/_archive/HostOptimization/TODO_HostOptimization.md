# TODO — HostOptimization

## 待办（建议下一轮）

1. **继续减 `backend()`**：按 `BACKEND_CALLSITE_INVENTORY` §C/D，优先 PluginGeometry / DocumentPointCloudOps / 轨迹页。
2. **Headless 共路加厚**：点云/几何/轨迹 ops 桌面调用收进 Bridge/Session（见 HEADLESS_OPS_ALIGNMENT）。
3. **Controller 继续按方法级拆文件**：勿整段匿名命名空间剪切；优先 Frames / Playback / Export。
4. **Web H2 / AiHost**：仅当有打包或隔离诉求时立项（OPTIONAL_EVAL）。

## 配置 / 环境

- 无新增 .env / API Key 需求。
- 构建：`MSBuild` 对 `CloudSimHost` / `RobotWidget` / `Widget`，Configuration=Debug|Release，Platform=x64。

## 需要你拍板时

- 是否立刻开下一波 `backend()` 大批迁移（工作量大）。
- 是否立项 Web 链接期去 OSG。
