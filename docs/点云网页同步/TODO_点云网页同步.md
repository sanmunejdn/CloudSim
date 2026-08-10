# TODO — 点云网页同步（后续）

## 需支持 / 配置

- 本地硬刷新后跑一遍 `ACCEPTANCE` 手工黄金路径（需样例 PLY/PCD）  
- 大点云（>50 万）验证 chunk 拼接与交互流畅度  

## 可选增强

1. **SSE `PointCloudJobProgress`**：长作业异步化，避免 HTTP 阻塞  
2. **Host 版本门控 UI**：SPARE/SDF/分阶段在旧 host 上禁用并提示  
3. **曲面分阶段预览对象**：分块着色网格 / 采样点云 / 拟合预览 B-rep（对齐桌面调试体验）  
4. **特征构建 / CAD 模板**：另开任务（本次共识不做）  
5. **几何 boolean 页**：从点云页移除后的独立入口  
6. **曲面高级参数表单**：当前多用默认 params，可对照桌面补控件  

## 已知小问题

- CloudSimWeb postbuild 偶发 `'LOUDSIM_WEB_FALLBACK'` 命令提示（不影响产物）  
- 多边形裁剪依赖视口 MVP/`cloudsimViewportPick`；若矩阵异常需对照 SceneViewport 暴露逻辑  
