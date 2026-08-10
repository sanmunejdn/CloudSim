# CONSENSUS — 点云网页同步

> 状态：已共识（2026-08-09）  
> 上游：`ALIGNMENT_点云网页同步.md`

## 1. 需求描述

将桌面 `PointCloudPlugin` **点云**侧栏能力同步到 Web 右坞「点云」页：UI 分区、Host 真作业、场景点云显示、多边形裁剪拾取；**暂不做**特征构建与 CAD 模板。

## 2. 范围

### 做

| 分区 | 能力 |
|------|------|
| 文档与导入 | 导入点云、刷新列表、当前文档标签 |
| 点云对象 / 信息 | 列表、选中、`queryPointCloudInfo` / measure 字段展示 |
| 下采样 | 体素 / 随机 |
| 裁剪 | 包围盒 / 球 / **多边形拾取** |
| 预处理 | 离群、双边平滑、法线 PCA、MST 定向（对齐桌面按钮集） |
| 配准 | ICP / SPARE / SDF·DDF |
| 重建网格 | Poisson Auto / Scale-space；网格下拉；导出 PLY |
| 网格后处理 | 简化 / Laplacian·Taubin / 修复 / 各向同性重网格 |
| 曲面重构 | 全流程 + 分阶段会话（≥1.12 / 1.13 门控） |
| 场景 | **混合渲染**：默认降采样 `THREE.Points`；点数 **> 500_000** 走 chunk LOD |
| UI 结构 | **单页纵向折叠分组**，一个「点云」Tab，顺序对齐桌面 `QGroupBox` |
| 清理 | 去掉点云页 stub 的 `boolean` / 伪算法下拉 |

### 暂不做

- 特征构建 Tab（管状铸件）
- CAD 模板 B-rep 更新（含面拾取）
- 独立「几何 / boolean」页（仅从点云页移除；另开任务）

### 不做

- 改 PointCloudAlgorithm 核心算法
- 部署/一网页一进程

## 3. 技术约束

- **真源**：`IPluginPointCloudHost` + Document（与桌面插件同 API），经 `CloudSimWebGateway` 暴露 REST + SSE。
- **禁止**：绕过 Host 直接散调算法 DLL 写回对象（避免与插件分叉）。
- **线程**：作业回调 / soup 导出走现有 GUI 线程排队模式（对齐 `/api/mesh`）。
- **前端**：`cloudsim-web-ui` React；替换 `GeometryOpsPanel` stub。
- **构建**：Web `build:debug` + `build:release`；若改 C++ Gateway/Host，相关 `.vcxproj` **Debug|x64 与 Release|x64** 均编。
- **版本门控**：polyline ≥1.11；曲面 ≥1.12 / 分阶段 ≥1.13；SPARE/SDF 按 host 版本隐藏或禁用。

## 4. 集成方案（摘要）

```
React PointCloudPanel (折叠分组)
  → REST /api/pointcloud/* (+ 拾取协议)
  → WebGateway → IPluginPointCloudHost / Document
  → SSE PointCloudJobProgress / Object* 事件
  → refreshObjects + SceneViewport 点云层（preview | chunk）
```

- 作业类：`POST` 带 `backendId` + 强类型 `params`，返回 `{ ok, jobId?, resultBackendId?, error?, metrics? }`；长任务用 SSE 进度。
- 预览：`GET /api/pointcloud/preview/:id?maxPoints=` → float32 xyz[+rgb]；超阈值则 `GET /api/pointcloud/chunk/:id?...` 分页/LOD。
- 多边形拾取：对齐轨迹拾取模式——前端画折线 → `POST` 闭合点列 → `cropPointCloudByPolyline`（或先 `pick` 会话再 crop，优先「前端采集 + 后端 crop」以适配无 Qt 视口）。

## 5. 验收标准

1. 右坞「点云」分区顺序与桌面一致（无 CAD / 无特征构建）。
2. 导入 PLY/PCD 等后列表可见点数；场景中可见点（小云预览 / 大云 chunk）。
3. 下采样、裁剪（含多边形）、预处理、ICP、重建、网格后处理、曲面全流程/分阶段：各至少一条黄金路径与桌面结果对象命名/类型一致（允许数值容差）。
4. SPARE/SDF 在 host 支持时可用，不支持时 UI 禁用并提示。
5. 作业进度可见（状态栏或分区内 progress）；失败有错误文案。
6. 点云页无 `boolean` stub；Debug/Release Web 构建通过；改动的 C++ 工程双配置通过。

## 6. 里程碑

| ID | 内容 |
|----|------|
| M1 | 导入/列表/信息 + 混合渲染骨架 + 下采样 + 预处理 |
| M2 | box/球裁剪 + ICP + 重建 + 导出 PLY |
| M3 | 网格后处理 + SPARE/SDF |
| M4 | 多边形拾取裁剪 |
| M5 | 曲面重构（全流程 + 分阶段 + 日志） |

## 7. 假设（默认可改）

- 预览/chunk 切换阈值默认 **500_000** 点（常量，可后续配置化）。
- 多边形拾取采用 **Web 视口采集折线 + REST crop**，不强制复用 Qt `pickPolylineFromViewport`（Headless 无 OSG 交互时更稳）。
- 桌面「预处理」四按钮一一映射；若 SDK 另有 `preprocessPointCloudForReconstruction`，可作为重建区可选快捷，不替代四按钮。
