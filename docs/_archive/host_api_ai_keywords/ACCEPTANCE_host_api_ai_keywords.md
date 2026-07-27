# ACCEPTANCE — Host 公共接口接入 AI（按钮 keywords + Agent 对话框）

## 功能验收

| # | 标准 | 结果 |
|---|------|------|
| 1 | 输入「体素下采样」→ rules 命中 `downsamplePointCloudVoxel` → 有选中/对话框选点云后执行 Host | 待手工 |
| 2 | 英文「Voxel downsample」同样命中 | 待手工 |
| 3 | 「点云匹配」/「ICP 配准」→ 弹出源+目标选择对话框 → `rigidRegisterPointCloudsIcp` | 待手工 |
| 4 | 「SPARE 配准」缺 target 时弹出双选对话框 | 待手工 |
| 5 | 「导入 PLY/XYZ…」无 path → 打开文件对话框 | 待手工 |
| 6 | 「导出 PLY…」无 path → 保存文件对话框 | 待手工 |
| 7 | 对话框点取消 → 可读失败摘要，不崩溃 | 待手工 |
| 8 | AiWidget 域下拉含：点云操作/文档导入/几何操作/特征构建/标注 | 代码已加 |
| 9 | `auto` 路由：含「点云/配准」→ `pointcloud.ops`；「点选边」→ `geometry.ops`（不误入标注） | 代码已加 |
| 10 | Catalog 与 `full_api_catalog.json` 同步（含 keywords） | 已嵌入 |
| 11 | 两份 DEVELOPER_GUIDE 已更新 | 已完成 |

## 编译

- 新增 AI 源文件（`AiHostButtonApiDispatch` / `AiAgentPickDialog` / `AiCatalogKeywordMatcher` / `CatalogActionPlanDomainHandler` / `AiApiCatalogEmbedded`）已通过 MSVC 编译
- 全量链接依赖环境中的 `RobotScene.lib` / `BackendVisual.lib` 路径问题时需本地先编依赖工程

## 手工步骤（建议）

1. 打开含至少 2 个点云的文档
2. AI 域选「自动」或「点云操作」，输入：`点云匹配`
3. 确认弹出源/目标对话框，勾选后执行
4. 输入：`体素下采样`，确认对选中或对话框所选点云下采样
