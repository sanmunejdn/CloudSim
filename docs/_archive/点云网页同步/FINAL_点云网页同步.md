# FINAL — 点云网页同步

## 总结

将桌面 `PointCloudPlugin` **点云**侧栏能力同步到 Web：真 REST（经 `HeadlessPointCloudBridge` → `document_point_cloud_ops` / `point_cloud_backend_ops`）、折叠分组 `PointCloudPanel`、场景混合渲染（50 万阈值）、多边形拾取裁剪、曲面全流程与分阶段会话。

**范围外（共识）**：特征构建、CAD 模板。

## 关键交付

| 层 | 路径 |
|----|------|
| Host 桥 | `Host/CloudSimHost/.../HeadlessPointCloudBridge.*` |
| Gateway | `Web/CloudSimWebGateway/source/WebGatewayPointCloud.cpp` |
| 前端面板 | `web/cloudsim-web-ui/src/docks/cloud/PointCloudPanel.tsx` |
| API | `web/.../api/pointcloud.ts` |
| 场景 | `SceneViewport` 加载 `geometryKind===1` |
| 文档 | `docs/点云网页同步/*` |

## 架构要点

- 不绕过 Host 直链算法 DLL；与桌面同一套 Data/算法路径  
- 作业当前为 GUI 线程同步完成（长作业会占住 HTTP；后续可改 jobId+SSE）  
- 分阶段曲面：会话驻于进程内 bridge；中间预览对象未全部注册（减负）  

## 构建

Debug/Release：`CloudSimHost`、`CloudSimWebGateway`、`CloudSimWeb`；Web `build:debug` + `build:release`。
