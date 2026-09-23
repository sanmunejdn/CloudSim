# 桌面 / 网页 Host 源清单同步

防止 `CloudSimHost` 与 `CloudSimHostHeadless` 共享源漂移（典型症状：网页 LNK2019，桌面正常）。

## 架构边界（刻意不对齐）

| 层 | 桌面 | 网页 |
|----|------|------|
| 宿主 DLL | `CloudSimHost` | `CloudSimHostHeadless`（`CLOUDSIM_HOST_HEADLESS_ONLY`） |
| 视口 | Qt + OSG Widget | Three.js；Host `render()` 为 Null |
| UI 能力 | 插件 Ribbon / Widget | `Headless*Bridge` → Gateway → React |
| 自测 | `PluginPropertyBindingSelfTest` | 不编 |

共享：`Data` / 几何与机器人内核、`DocumentHost` 编排、`CloudSimPluginHost` 业务实现（除桌面专用 UI）。

**不**把共享 `ClCompile` 抽到单独 `.props`：`generate_vcxproj_filters.py` 只解析 `.vcxproj` 本体，Import 的 ItemGroup 会被漏扫。

## 工程同步（必做）

在 `CloudSim/` 根目录：

```bash
# 检查（退出码 1 = 有漂移）
python scripts/check_host_headless_sources.py

# 自动把「共享源仅在桌面」写入 Headless.vcxproj
python scripts/check_host_headless_sources.py --fix

# 更新筛选器
python scripts/generate_vcxproj_filters.py --sync --project CloudSimHostHeadless
```

然后 **Debug|x64 与 Release|x64** 各编一遍：

```text
msbuild src\Host\CloudSimHost\CloudSimHostHeadless.vcxproj /p:Configuration=Debug /p:Platform=x64
msbuild src\Host\CloudSimHost\CloudSimHostHeadless.vcxproj /p:Configuration=Release /p:Platform=x64
```

Allowlist 查看：`python scripts/check_host_headless_sources.py --list-allow`  
（桌面 OSG/Widget UI、SelfTest；Headless 的 `OsgWidgetSceneBridge_Headless.cpp`。）

新增 **桌面专用** 源时：把 basename 加入脚本内 `DESKTOP_ONLY_BASENAMES`，否则 `--fix` 会误写入网页工程。

## 能力对等同步（产品层）

1. 桌面金标：标清 Data 真源 / Host 自由函数 / 仅 UI  
2. Host：复用编排；缺则加 `HeadlessXxxBridge`，禁止复制算法  
3. Gateway：`WebGateway<Domain>.cpp` + [`网页端全量对等/API_CONTRACT.md`](../网页端全量对等/API_CONTRACT.md)  
4. 前端：`web/cloudsim-web-ui`；SSE 与桌面事件语义对齐  

验收：同一工程包两端打开，关键 JSON 字段一致。

刻意不对等：PlaneGCS 视口草图、DatumPlane overlay、OSG 边面拾取、桌面 CommandStack 实现细节。

## 发布同步

- 前端：`npm run build:debug|release` → 仓库根 `bin\x64d\web` / `bin\x64\web`  
- 正式包勿用 `CLOUDSIM_WEB_FALLBACK=1`
