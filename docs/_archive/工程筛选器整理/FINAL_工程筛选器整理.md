# FINAL — 工程筛选器整理

## 总结

已按共识对 `CloudSim.sln` **全部 39 个工程**的 Solution Explorer 筛选器做功能细分：磁盘子目录优先镜像，扁平工程按文件名/职责聚类；跨工程与 SDK 源归入 `External\<相对路径>`。

## 交付物

| 路径 | 说明 |
|------|------|
| `tools/RegenerateProjectFilters.ps1` | 可重复生成脚本（规则即文档） |
| 各工程 `*.vcxproj.filters` | 已更新或新建 |
| `docs/工程筛选器整理/*` | ALIGNMENT / CONSENSUS / DESIGN / TASK / ACCEPTANCE / FINAL / TODO |

## 使用说明

重新生成（例如增删源文件后）：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File CloudSim\tools\RegenerateProjectFilters.ps1 -Root CloudSim
# 或只跑指定工程：
# ... -OnlyProjects Widget,RobotWidget
```

在 Visual Studio 中若筛选器未刷新：关闭重开解决方案，或卸载/重新加载工程。

## 质量说明

- 只改 `.filters`，不影响编译
- 归类规则集中在脚本 `Get-FunctionalBucket` / `Get-DiskMirrorFilter`，后续调规则后重跑即可
