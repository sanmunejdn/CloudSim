# ACCEPTANCE — 点云网页同步

> 状态：实现完成，待手工黄金路径点验  
> 构建：CloudSimHost / CloudSimWebGateway / CloudSimWeb **Debug|x64 + Release|x64** 通过；`npm run build:debug|release` 通过

## 验收对照（CONSENSUS §5）

| # | 标准 | 状态 |
|---|------|------|
| 1 | 右坞「点云」分区顺序对齐桌面（无 CAD / 无特征构建） | ✅ UI 已落地 |
| 2 | 导入后列表有点数；场景可见点（≤50 万 preview / 超阈值 chunk） | ✅ API+SceneViewport |
| 3 | 下采样 / 裁剪（盒/球/多边形）/ 预处理 / ICP / 重建 / 网格后处理 / 曲面全流程·分阶段 | ✅ REST+面板；待手工点验 |
| 3b | 参数 UI/默认值对齐桌面（体素 2、球 r=50、预滤波 1、后处理面数/λ/填孔等） | ✅ 2026-08-09 面板已对齐 |
| 4 | SPARE/SDF 方法下拉 + 选项（体素预滤波/场模式/刚性预对齐） | ✅；「输出为新对象」网页暂就地变形并提示 |
| 5 | 作业进度/失败可见 | ⚠️ 同步 HTTP + 状态栏；无 SSE 细粒度进度 |
| 6 | 无 boolean stub；双配置构建通过 | ✅ |

## 手工清单（请本地跑一遍）

1. 启动 `bin\x64d\CloudSimWeb.exe`，硬刷新浏览器  
2. 导入 PLY/PCD → 列表选中 → 场景见点  
3. 体素下采样 → 点数下降  
4. 盒/球/多边形裁剪  
5. 法线 PCA → 信息「法线 有」  
6. ICP（两朵点云）  
7. Poisson Auto → 网格 → 简化/平滑  
8. 曲面：全流程 或 预处理→…→装配  

## 明确不做（共识）

- 特征构建 Tab  
- CAD 模板 B-rep 更新  
- 独立几何 boolean 页  
