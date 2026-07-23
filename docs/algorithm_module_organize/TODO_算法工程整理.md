# TODO — 算法工程整理

## 需你本地确认

1. **VS 编译** `PointCloudAlgorithm` → `Data`（验证 `Reconstruction.h` 转发与新 cpp 入工程）
2. 可选跑 `pclalgo::runSelfTest` / `vcgalgo::runSelfTest`

## 可选后续（未排期）

| 项 | 说明 |
|----|------|
| Preprocess 再拆 | 若仍觉混杂：法线 / 离群平滑 / 重建前管线分文件，旧头转发 |
| Geometry Params 字段 | `MeshSurfaceReconstructParams` 等大结构体可逐字段 `///<` |
| 编码脚本 | 仓库级 `normalize_source_encoding.py` 无路径参数；本轮已对改动目录做定向 BOM+CRLF |
| GUIDE 长文 | Geometry `DEVELOPER_GUIDE` 流水线章节未整章重写（仅依赖公开头自文档） |

## 缺配置

无新增第三方依赖或环境变量。
