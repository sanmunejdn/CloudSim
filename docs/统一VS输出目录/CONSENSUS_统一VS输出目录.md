# CONSENSUS — 统一 VS 输出目录

## 需求与验收

1. 所有 CloudSim 产品工程 Debug|x64 / Release|x64 的 `OutDir`/`IntDir` 使用 `$(CloudSimBinDir)` / `$(CloudSimIntRoot)<工程名>\`（由 `Directory.Build.props` 解析到仓库根 `bin\x64` / `x64d` 与 `x64middle` / `x64dmiddle`）。
2. 插件 `OutDir` = `$(CloudSimBinDir)plugins\<plugin.id>\`；中间目录仍在 IntRoot 下按工程名。
3. 保留现有 `plugin.json` / 配置 / 第三方 DLL 的 PostBuild。
4. 删除误生成的 `CloudSim\src\Plugins\bin\`。
5. 不改动 x86 配置。

## 技术约束

- 禁止在 vcxproj 写死 `D:\Project\...` 绝对路径。
- 禁止依赖 `$(SolutionDir)../bin` 作为 x64 主输出路径。
- 不改第三方 SDK 布局与运行时查找逻辑。

## 已确认决策

| 项 | 选择 |
|----|------|
| Q1 PostBuild | 2 — 只统一目录，保留 sidecar/第三方复制 |
| Q2 范围 | 1+3 — 全量产品工程 + 清 Plugins\bin |
| Q3 x86 | 1 — 不动 |
