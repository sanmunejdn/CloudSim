# GeometryServices 模块

> **文档导航**：[全库入口](../../../../docs/README.md) · [全量目录](../../../../docs/全量目录.md) · [开发手册](../../../../docs/开发手册/01-总览.md) · [产品索引](../../../docs/README.md) · [模块总表](../../../docs/MODULE_DEVELOPER_GUIDES.md)

算法编排门面自 Data 模块迁出，链接 `Data` 与 `GeometryAlgorithm`。

## 头文件

| 头文件 | 命名空间 | 说明 |
|--------|----------|------|
| `GeometryBackendOps.h` | `geometry_backend_ops` | B-rep 离散、模板配准、曲面/管磨重构等 |
| `GeometryRef.h` | `geometry_backend_ops` | 特征轨迹 `GeometryRef`、特征列表编解码 |
| `PointCloudBackendOps.h` | `point_cloud_backend_ops` | 点云裁剪/配准/重建与网格后处理 |
| `MeshBoolean.h` | `MeshBoolean` | 三角 soup 布尔运算 |

消费方在 include 路径中加入 `../../Geometry/GeometryServices/inc`（或工程已配置），并链接 `GeometryServices.lib`。

## 依赖

- `Data.lib` — 后端数据类型
- `GeometryAlgorithm.lib` / `PointCloudAlgorithm.lib` / `VcgAlgorithms.lib`

详见原 Data `DEVELOPER_GUIDE.md` §4.8–4.10（已迁出）。
