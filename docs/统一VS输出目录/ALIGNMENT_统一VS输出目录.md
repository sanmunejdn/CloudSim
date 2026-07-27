# ALIGNMENT — 统一 VS 工程 Debug/Release 输出目录

## 原始需求

检查所有 VS 工程在 Debug/Release 的输出目录（特别是插件），统一用相对路径指向仓库根下：

- 输出：`D:\Project\VSprogram\CGAL5.5.2\bin\x64`（Release）、`...\bin\x64d`（Debug）
- 中间：`...\bin\x64middle\<工程名>\`、`...\bin\x64dmiddle\<工程名>\`
- 取消「编译到别处再复制到 bin」的做法，改为直接 `OutDir` 到目标目录

## 项目现状

### 已有统一变量（正确基准）

`CloudSim/Directory.Build.props`：

| 变量 | Debug\|x64 | Release\|x64 |
|------|------------|--------------|
| `CloudSimBinDir` | `$(CloudSimRepoRoot)bin\x64d\` | `$(CloudSimRepoRoot)bin\x64\` |
| `CloudSimIntRoot` | `...\bin\x64dmiddle\` | `...\bin\x64middle\` |

`CloudSimRepoRoot` = `Directory.Build.props` 所在目录的上一级 = 仓库根 `CGAL5.5.2\`。  
**不依赖 `$(SolutionDir)`**，单独打开 `.vcxproj` 也能落到同一绝对路径。

### 问题根因

大量工程仍写死：

```xml
<OutDir>$(SolutionDir)../bin/x64d/</OutDir>
```

当从 `src/Plugins` 下单独编、或 `SolutionDir` 不是 `CloudSim\` 时，会落到：

`CloudSim\src\Plugins\bin\x64(d)\`（已实测存在 `CloudSimPluginSDK.dll` 等产物）

而不是目标 `CGAL5.5.2\bin\x64(d)\`。

### 工程分类（x64）

| 风格 | 代表工程 | 风险 |
|------|----------|------|
| 已用 `$(CloudSimBinDir)` / `$(CloudSimIntRoot)` | Data、CloudSimHost、GeometricModelingPlugin、ProcessFlowPlugin、Plc\*、IndustrialCamera\*、RobotCommSDK、部分 Geometry 等 | 低 |
| 仍用 `$(SolutionDir)../bin/...` | CloudSimPluginSDK、Labeling/PointNet/PointCloud/Geometry/HelloAi 插件、多数 UI/Robot/部分 Geometry 等 | **高**（易写到错误 bin） |
| 用 `$(ProjectDir)../../../../bin/...` | CloudSimCore、CloudSimBootstrap、tools/SketchExtrudeDraftVerify | 中（深度绑死，不如 props） |

### 当前 PostBuild「复制」清单（非 DLL 产物搬迁）

| 类型 | 工程 | 说明 |
|------|------|------|
| 插件清单/配置 | 各 `*Plugin` | `plugin.json`（及部分 `*_config.json`）→ `$(OutDir)` |
| 资源 | IndustrialCameraPlugin、GeometryAlgorithm | 资源/README 等到 bin |
| 第三方运行时 | IndustrialCameraSDK、PlcCommSDK | MechEye/OpenCV/plctag 等从 `bin\SDK` 拷到运行目录 |

**未发现**「本工程 DLL 先编到 A 目录再 copy 到 bin」的 PostBuild；错位主要来自错误的 `OutDir`/`SolutionDir`。

## 边界确认（任务范围）

**纳入：**

- `CloudSim` 下全部产品 `.vcxproj` 的 Debug\|x64 / Release\|x64：`OutDir`/`IntDir` 统一为 `$(CloudSimBinDir)` / `$(CloudSimIntRoot)<工程名>\`
- 插件：`OutDir` = `$(CloudSimBinDir)plugins\<plugin.id>\`，`IntDir` 仍在 middle 下按工程名
- 去掉各工程里对 `CloudSimBinDir`/`CloudSimIntRoot` 的重复本地兜底（若与 props 重复）
- 可选：清理误生成的 `CloudSim\src\Plugins\bin\`（构建产物，非源码）

**不纳入（除非另确认）：**

- x86 配置（多数工程仍有，但产品主路径是 x64）
- 改动第三方 SDK 路径、改运行时查找逻辑
- 删除「第三方 DLL / plugin.json」类 PostBuild（见疑问）

## 需求理解

1. 「相对路径」= 相对仓库根的 MSBuild 属性（`$(CloudSimRepoRoot)` / `$(CloudSimBinDir)`），解析结果等于上述绝对路径；**不要**在 vcxproj 写死 `D:\Project\...`。
2. 「取消编译后再复制」= 产物直接链出到 `bin\x64(d)`（及插件子目录），不再依赖 `SolutionDir` 间接路径或二次搬迁。

## 疑问澄清（待确认）

### Q1 — PostBuild 复制 sidecar / 第三方 DLL（关键）

现有 PostBuild 主要是：

- A. `plugin.json` / 配置文件 → 插件 OutDir  
- B. 第三方运行时 DLL（MechEye、plctag 等）→ 运行目录  

是否：

1. **全部保留**（只改 OutDir/IntDir）— 推荐，否则插件加载会缺 `plugin.json`
2. 仅取消「若存在的本工程二进制二次拷贝」（当前基本没有）；A/B 保留
3. 连 A/B 也去掉（需改用 `Content`+`CopyToOutputDirectory` 或其它部署方式）

### Q2 — 范围

1. **全部 CloudSim 产品工程**（含 App/UI/Geometry/Robot/Plugins/tools）— 推荐  
2. 仅 `src/Plugins`  
3. 全部 + 顺带删掉误生成的 `src/Plugins/bin`

### Q3 — x86 配置

1. 本次不动 x86  
2. x86 也改成相对 `CloudSimRepoRoot` 的 `bin\x86(d)`（若仍需要）

## 建议默认决策（若你回复「按推荐执行」）

- Q1 → **2**（只统一 OutDir/IntDir；保留 plugin.json / 第三方 DLL PostBuild）  
- Q2 → **1+3**（全量产品工程 + 清理 `Plugins\bin`）  
- Q3 → **1**（不动 x86）
