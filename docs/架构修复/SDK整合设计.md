# SDK 整合设计（核心 PluginSDK + 域扩展）

> 立项来源：[`P2_方向_中级缺陷.md`](P2_方向_中级缺陷.md) ⑨「长期：8 个 SDK → 核心 PluginSDK + 域扩展 DLL，立项评估」。
> 本文给出依赖矩阵、两级结构、迁移顺序与 ABI 影响；Labeling 试点结论见第 6 节。
>
> **终态（W3）：`CloudSimLabelingSDK` 已物理退场**——工程已从 `CloudSim.sln` / `CloudSimWeb.sln` 移除，目录已删除。  
> **标注会话 = HostCore `LabelingSession`**（`CloudSimHostCore`：`inc/pluginhost/LabelingSession.h` + `source/pluginhost/LabelingSession.cpp`）。  
> 插件面标注 ABI 仍在 PluginSDK（`PluginLabelingTypes.h` + `IPluginLabelingHost.h`）。下文历史盘点中出现的 LabelingSDK 行保留作迁移对照，以本节终态为准。

## 1. 现状盘点

`CloudSim/src/Plugins/` 下现存 SDK 均为独立 vcxproj + DLL（**不含已移除的 LabelingSDK**）：

| SDK | 公开头文件 | 定位（各 DEVELOPER_GUIDE） |
|-----|-----------|---------------------------|
| **CloudSimPluginSDK** | `ICloudSimPlugin.h`、`IPluginHostContext.h`、`IPluginDocument.h`、`IPluginPointCloudHost.h`、`IPluginGeometryHost.h`、`IPluginLabelingHost.h`、`IPluginRobotHost.h`、`IPluginSceneBridge.h`、`IProcessFlowAiBridge.h`、`Plugin*Types.h`（PointCloud/Geometry/Labeling/Robot/Primitive/BackendMeta）、`CloudSimPluginVersion.h`、`cloudsim_plugin_sdk_global.h`（17 个） | 插件与宿主之间**唯一稳定 ABI**；SDK/IID `0x00013900`（含 `ICloudSimPlugin::sdkAbiVersion()`） |
| **CloudSimAiSDK** | `ICloudSimAiPlugin.h`、`IAiAssistantHost.h`、`IAiInferenceProvider.h`、`IAiDomainHandler.h`、`Ai*Types.h`/`AiConfigDto.h` 等 DTO（13 个） | AI 插件二级接口 + DTO；IID `com.cloudsim.ICloudSimAiPlugin/1.0.0x00013900`（与核心 SDK 同步 bump） |
| ~~**CloudSimLabelingSDK**~~（**已移除**） | 原 `LabelingSession.h` / `LabelingTypes.h` | **迁入 HostCore**；无独立 DLL / vcxproj |
| **CloudSimMeshTrajectorySDK** | `MeshTrajectorySession.h`、`MeshTrajectoryTypes.h`、`mesh_trajectory_sdk_global.h`（3 个） | 无 Qt mesh 轨迹会话；**依赖 GeometryAlgorithm 头**（`geoalgo::MeshTrajectorySpec`） |
| **CloudSimControllerSDK** | `IControllerClient.h`、`ControllerTypes.h`、global（3 个） | 外置控制器客户端（localhost TCP JSON :19620） |
| **RobotCommSDK** | `IRobotMotionClient.h`、`RobotCommTypes.h`、global（3 个） | 真机通讯客户端（localhost TCP JSON :19610） |
| **PlcCommSDK** | `IPlcCommClient.h`、`PlcCommTypes.h`、`PlcTagStringBuilder.h`、global（4 个） | PLC 通讯（libplctag：Modbus TCP / AB EIP） |
| **IndustrialCameraSDK** | `ICamera.h`、`CameraTypes.h`、`HandEyeTypes.h`、`BoardDetector.h`、`MechOfficialHandEye.h`、`IRobotPoseSource.h`、global（7 个） | 工业相机（海康/梅卡曼德 stub + OpenCV + 手眼标定） |

## 2. 依赖矩阵（消费者 × SDK）

来源：各工程 `.vcxproj` 的 `AdditionalDependencies`（链接 `.lib`）与 `ProjectReference`（工程引用），并以源码 `#include` 交叉验证。

| 消费者工程 | PluginSDK | AiSDK | MeshTrajectorySDK | ControllerSDK | IndustrialCameraSDK | PlcCommSDK | RobotCommSDK |
|------------|:---------:|:-----:|:-----------------:|:-------------:|:-------------------:|:----------:|:------------:|
| CloudSimHost / Headless | ● | ● | | | | | |
| CloudSimPluginHost (UI) | ● | ● | | | | | |
| GeometryPlugin | ● | | | | | | |
| GeometricModelingPlugin | ● | | | | | | |
| EngineeringDrawingPlugin | ● | | | | | | |
| PointCloudPlugin | ● | | | | | | |
| ProcessFlowPlugin | ● | | | | | | |
| HelloAiPlugin | ● | ● | | | | | |
| PointNetPlugin | ● | ● | | | | | |
| LabelingPlugin | ● | ● | | | | | |
| IndustrialCameraPlugin | ● | | | | ● | | |
| PlcCommPlugin | ●（经 PlcCommUI 间接用 PlcCommSDK） | | | | | | |
| PlcCommUI | | | | | | ● | |
| AiWidget (UI) | ● | ● | | | | | |
| RobotWidget (UI) | | | ● | | | | ● |
| CloudSimControllerSample | | | | ● | | | |

标注会话：`CloudSimHostCore` 内编 `LabelingSession`（无独立 LabelingSDK 链接）。`PluginLabelingHostImpl` `#include "LabelingSession.h"`（HostCore 头）。

源码级真实消费关系：

- ~~`CloudSimLabelingSDK`~~：**已移除**。原唯一源码消费者是宿主 `PluginLabelingHostImpl`；现直接使用 HostCore `LabelingSession`。
- `CloudSimMeshTrajectorySDK` 仅被 `RobotWidget`（UI）消费，包装 `geoalgo::generateMeshTrajectory`，不属于插件 ABI。
- Controller / RobotComm / PlcComm / IndustrialCamera 四个 SDK 是**外部通讯/厂商封装客户端**，与插件 ABI 正交。

## 3. 目标两级结构

```mermaid
graph TD
    subgraph 核心级（插件 ABI，唯一稳定面）
        CORE[CloudSimPluginSDK<br/>ICloudSimPlugin / IPluginHostContext<br/>IPlugin*Host / Plugin*Types<br/>IID 编码版本号]
    end
    subgraph 宿主内部（非独立 SDK）
        LABEL[HostCore LabelingSession<br/>标注会话引擎]
    end
    subgraph 域扩展级（按需链接，不进核心 ABI）
        AI[CloudSimAiSDK<br/>AI 插件二级接口 + DTO]
        MESH[CloudSimMeshTrajectorySDK<br/>UI 内部轨迹会话]
        EXT[外部通讯客户端 SDK<br/>Controller / RobotComm<br/>PlcComm / IndustrialCamera]
    end
    HOST[CloudSimHost / PluginHost] --> CORE
    HOST --> LABEL
    UI[RobotWidget / AiWidget / PlcCommUI] --> MESH
    UI --> EXT
    PLUGIN[各插件 DLL] --> CORE
    PLUGIN -.按需.-> AI
    AIP[HelloAi/PointNet/Labeling 插件] --> AI
    CAM[IndustrialCameraPlugin] --> EXT
```

两级划分的判据：

1. **进核心级（PluginSDK）的唯一理由**是「插件与宿主跨 DLL 边界的契约」：接口（纯虚，无实现）、POD 类型、版本握手。核心级头文件必须无 Qt 以外依赖、无第三方库依赖、无导出符号实现（接口 + 内联 POD）。
2. **域扩展级**保持独立 DLL，按域分三类管理：
   - *插件面域 ABI*（AiSDK）：插件直接实现/消费的二级契约，保留独立 DLL，但 IID 策略向核心对齐（见 §5）。
   - *内部逻辑*：标注会话已编入 HostCore（**非独立 SDK**）；MeshTrajectorySDK 仍为 UI 内部 DLL；**不上移 PluginSDK**，避免核心级携带实现符号与 GeometryAlgorithm 依赖。
   - *外部通讯客户端*（Controller/RobotComm/PlcComm/IndustrialCamera）：自带厂商依赖（libplctag、Hik/Mech-Eye、OpenCV、Winsock），永远独立，不并入核心。

## 4. 迁移顺序

| 阶段 | 内容 | 联动重编范围 |
|------|------|--------------|
| 0（本轮） | 本设计文档；持久化 `minReaderVersion` 版本协商（`BackendDataBase`）；**不动任何 vcxproj / 头文件位置** | 仅 Data |
| 1 | Labeling 去重：LabelingPlugin 移除对 `CloudSimLabelingSDK.lib` 的 vestigial 链接；训练 POD 单源化；`LabelingSession` 迁入 HostCore | **已完成** |
| 1b（W3） | **物理退场 LabelingSDK**：从 sln 移除、删除目录；文档改为 HostCore `LabelingSession` | **已完成** |
| 2 | AiSDK IID 对齐：`CloudSimAiPlugin_iid` 版本后缀编码（同 `CloudSimPlugin_iid` 模式） | **已完成**（`0x00013900` 与核心 SDK lockstep） |
| 3 | MeshTrajectorySDK 定位固化：UI 内部库，非插件 ABI | **已完成**（文档；插件禁直链） |
| 4 | 外部通讯 SDK 保持独立；仅整理文档索引与命名约定 | 无 |

每阶段独立成 PR；阶段 1/2 涉及插件与宿主联动重编，须安排统一编译窗口（避免并行任务锁 `bin` 输出）。

## 5. ABI 影响

1. **核心 IID 已编码版本号**：`CloudSimPlugin_iid = "com.cloudsim.ICloudSimPlugin/1.0." CLOUDSIM_PLUGIN_SDK_VERSION_STR`（当前 `0x00013900`）。宿主在 metaData IID 校验后调用 `ICloudSimPlugin::sdkAbiVersion()` 与 `cloudsimPluginSdkVersion()` 二次握手。**任何核心接口改动必须 bump `CLOUDSIM_PLUGIN_SDK_VERSION` 并重编全部插件 DLL**。
2. **虚函数只能末尾追加**：`IPluginPointCloudHost` 等接口在中间插入虚函数会导致插件与宿主 vtable 错位（已有 `queryMeshInfo` 误调 `simplifyMesh` 崩溃的先例，见 PluginSDK DEVELOPER_GUIDE）。整合时禁止重排既有虚函数顺序。
3. **AiSDK IID 已编码版本后缀**（`com.cloudsim.ICloudSimAiPlugin/1.0.0x00013900`）：与核心 PluginSDK 同步 bump；`check_plugin_iid_alignment.py` 校验双真源一致。
4. **域 SDK 的 dllexport 类**：`LabelingSession` 已编入 HostCore（不再经独立 LabelingSDK DLL 导出）；`MeshTrajectorySession`（依赖 `geoalgo::` 类型）仍是带实现的导出类。头文件物理移动不改变符号所在 DLL；若把实现改挂到别的 DLL，所有消费方必须重编且部署新 DLL 集。
5. **持久化版本协商与插件 ABI 正交**：本轮新增的 `minReaderVersion`（工程 JSON）是文件格式协商，不影响任何 DLL 边界。

## 6. Labeling 试点结论：已迁 HostCore 并物理退场

任务预设的试点条件是「LabelingSDK 公开接口纯头文件 / 可内联进 PluginSDK」。逐头核实后**条件不成立**（历史结论）；正确路径是**迁入 HostCore + 删除独立工程**，而非上移 PluginSDK：

| 头文件 | 性质 | 终态 |
|--------|------|------|
| `LabelingTypes.h` | 纯 POD，无 Qt | 随 `LabelingSession` 在 HostCore；插件面用 PluginSDK `PluginLabelingTypes.h` |
| `LabelingSession.h` | 导出类 + `.cpp` 实现 | **HostCore**（`inc/pluginhost/`、`source/pluginhost/`） |
| `labeling_sdk_global.h` | 导出宏 | 随工程删除 |

更根本的原因：**插件面标注 ABI 早已内化进 PluginSDK**（`PluginLabelingTypes.h` + `IPluginLabelingHost.h`，v1.16.0+，经 `IPluginHostContext::labelingHost()` 获取）。独立 `CloudSimLabelingSDK.dll` 仅为宿主内部会话引擎外壳，已无存在必要——**W3 已物理删除工程与目录**。

## 7. 风险

| 风险 | 说明 | 缓解 |
|------|------|------|
| 联动重编 | 核心 SDK 头变更 → 宿主 + 全部插件重编；并行开发时 `bin` 输出被锁 | 阶段 1/2 安排统一编译窗口；每阶段独立 PR |
| 类型语义漂移 | Labeling 三处 POD 字段曾不一致（`best`/`isBest`、`std::string`/`QString`），单源化时须逐一核对消费点 | 阶段 1 已单源化到 PluginSDK / HostCore |
| Headless 失步 | `CloudSimHost` 与 `CloudSimHostHeadless` 共享源，SDK 结构调整须同步两个 vcxproj | 改后跑 `scripts/check_host_headless_sources.py` |
| 厂商依赖 | IndustrialCameraSDK / PlcCommSDK 携带 Hik/Mech-Eye/OpenCV/libplctag，无法纯头文件化、不可并入核心 | 永久归类域扩展级，文档固化 |
| AI IID 无版本保护 | 阶段性风险；阶段 2 已对齐 IID 版本编码 | 已完成 |
| 残留引用 | 旧文档 / 脚本仍写 LabelingSDK 路径 | W3 已清 sln、目录、索引文档；构建前 grep `CloudSimLabelingSDK` |
