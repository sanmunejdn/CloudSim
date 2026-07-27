# FINAL — 统一 VS 输出目录

## 做了什么

将 CloudSim 全部产品 `.vcxproj` 的 Debug|x64 / Release|x64 输出与中间目录，统一为 `Directory.Build.props` 中的：

- `$(CloudSimBinDir)` → 仓库根 `bin\x64d\` / `bin\x64\`
- `$(CloudSimIntRoot)<工程名>\` → `bin\x64dmiddle\` / `bin\x64middle\`
- 插件：`$(CloudSimBinDir)plugins\<plugin.id>\`

去掉对 `$(SolutionDir)../bin` / `$(ProjectDir).../bin` 作为 x64 主路径的依赖；删除误生成的 `src/Plugins/bin`；保留 sidecar / 第三方 DLL PostBuild。

## 关键修正

- `CloudSim.vcxproj` IntDir 补全为 `CloudSim\`
- 去掉 GeometryAlgorithm / InstantMeshes* 中被替换后失效的本地 `CloudSimBinDir` 兜底块
- 插件 OutDir 尾部分隔符统一为 `\`

## 验证

抽样三工程 Debug+Release 均输出到 `D:\Project\VSprogram\CGAL5.5.2\bin\...`，未再写入 `Plugins\bin`。
