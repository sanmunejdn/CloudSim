# ACCEPTANCE — 统一 VS 输出目录

## 子任务完成情况

| 任务 | 状态 | 证据 |
|------|------|------|
| T1 批量改写 OutDir/IntDir | 通过 | 全量产品工程 x64 均用 `$(CloudSimBinDir)` / `$(CloudSimIntRoot)`；插件带 `plugins\<id>\` |
| T2 删除 Plugins\bin | 通过 | `CloudSim\src\Plugins\bin` 已删除；抽样编译后未再生 |
| T3 抽样编译 | 通过 | 见下 |

## 抽样编译（Debug\|x64 + Release\|x64）

目标：`CloudSimPluginSDK`、`GeometryPlugin`、`PointCloudPlugin`

| 产物 | Debug | Release |
|------|-------|---------|
| `bin\x64(d)\CloudSimPluginSDK.dll` | OK | OK |
| `bin\x64(d)\plugins\com.cloudsim.geometry\GeometryPlugin.dll` | OK | OK |
| `bin\x64(d)\plugins\com.cloudsim.pointcloud\PointCloudPlugin.dll` | OK | OK |
| 中间目录 `bin\x64(d)middle\CloudSimPluginSDK` | OK | OK |

## 整体验收

- [x] 需求：统一到仓库根 `bin\x64` / `x64d`
- [x] 插件直接输出到 `plugins\<id>\`
- [x] 保留 plugin.json 等 PostBuild
- [x] x86 未改
- [x] 误目录已清
- [x] 抽样 Debug+Release 编译通过
- [x] SDK `AdditionalLibraryDirectories` 改为 `$(CloudSimRepoRoot)bin\SDK\...`（含相关 PostBuild/Copy）
- [x] `AdditionalIncludeDirectories` / qtpropertybrowser 源文件 / RobotWidget Python 部署路径全部统一到 `CloudSimRepoRoot` / `OutDir`
