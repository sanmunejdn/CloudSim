# Registry 去单例设计（ServiceRegistry 化）

> 立项来源：[`P2_方向_中级缺陷.md`](P2_方向_中级缺陷.md) 架构缺陷分级修复，工作流 D。
> 本文记录 5 个 Registry 的处置决策与后续迁移路线；本轮已落地代码见 §4。

## 1. 目标与约束

- 引入 `cloudsim::core::ServiceRegistry`（非单例），由 `ICloudSimContext` 持有，作为各 Registry 的最终持有点。
- 本轮**不改** `::instance()` 调用点（全仓库 30+ 处，跨 6+ 工程），只改 Registry 类本身。
- 不得引入反向 DLL 依赖（Data / TrajectoryAlgorithm / GeometryAlgorithm 不得链接 CloudSimHost）。

## 2. 关键事实（决策依据）

1. `cloudsimApplicationContext()` 由 **CloudSimHost.dll** 导出（`CloudSimBootstrap.h`，`CLOUDSIM_HOST_EXPORT`）。
2. 依赖方向：`CloudSimHost.dll → Data.dll / TrajectoryAlgorithm.dll / GeometryAlgorithm.dll / BackendVisual.dll`。下层 DLL 调用 `cloudsimApplicationContext()` 即构成反向依赖，不可行。
3. 在 CloudSimCore 新增全局上下文访问器 = 再造一个全局单例，违背目标，不做。
4. `BackendVisualRegistry` 无实例状态：全部静态方法，工厂表在 `BackendVisualRegistry.cpp` 匿名命名空间。改实例化必须改写全部静态调用点，超出本轮边界。

## 3. 处置决策

| Registry | 所在模块 | 决策 | 理由 |
|----------|----------|------|------|
| `BackendComponentCodecRegistry` | Data.dll | 构造改 public；`instance()` 保留静态兜底 | DLL 无法访问宿主上下文（事实 2） |
| `TrajectoryOpRegistry` | TrajectoryAlgorithm.dll | 同上 | 同上 |
| `FeatureDiscretizerRegistry` | GeometryAlgorithm.dll | 同上 | 同上 |
| `GeometryFileImporterRegistry` | CloudSimHost.dll / Headless.dll | **完整落地**：构造/析构 public；`instance()` 优先 ServiceRegistry、回退静态实例；组合根构造时注册 | 与上下文同模块，无反向依赖 |
| `BackendVisualRegistry` | BackendVisual.dll | **保留单例不变**（有意的全局状态） | 事实 4；头文件已加 @note 说明 |

### GeometryFileImporterRegistry 的安全性论证

- 唯一 `add()` 调用方是 `registerBuiltinGeometryImporters`，由 `ensureBuiltinsRegistered()` 惰性触发，不存在外部自定义导入器注册路径。
- 上下文创建（`main` 首行 `cloudsimSetApplicationContext`）先于一切导入操作，`instance()` 稳定返回 ServiceRegistry 持有实例。
- 极端时序（上下文创建前调用 `instance()`）只会在兜底实例上再注册一次内置导入器，无状态丢失，仅一份冗余内存。

## 4. 本轮已落地

- `ICloudSimContext` 新增 `services()`（const/non-const）；`ApplicationContextImpl` 持有 `ServiceRegistry m_services` 并在构造时注册 `GeometryFileImporterRegistry`。
- 4 个 Registry 构造（含 `GeometryFileImporterRegistry` 析构）改 public，注释说明 Why。
- `instance()` 一律保留，注释标记为兼容期入口。

### 关于 `[[deprecated]]` 的决策

未加属性，只用注释标记。理由：调用点本轮不迁移，加属性会在 6+ 工程产生 30+ 处 C4996 警告噪音，淹没编译验证；待调用点迁移启动时再补 `[[deprecated]]`。

## 5. Window 8（调用点 / 组合根注入，部分落地）

| 项 | 状态 |
|----|------|
| R1 `BackendRegistry` / `BackendComponentCodecRegistry` 进 `ServiceRegistry` + `setProcessInstance` | 已落地；Host 经 `backendRegistry()` 访问 |
| R2 `makeBackendManagerDataService(BackendDataManager*)` 可选注入 | 已落地；`DocumentHost` 仍用 `DataServiceAdapter`，导出 API 默认 `instance()` |
| R3 `TrajectoryOpRegistry` / `FeatureDiscretizerRegistry` 组合根注册 + Bridge `set*Registry` | 已落地 |
| R5 `BackendVisualRegistry` | 仍保留静态工厂表，待 BackendVisual 重构一并排期 |

## 6. 后续迁移路线

**清债关门（#8 / D2+D6）已完成主干：**

1. Importer / Backend / Codec / TrajectoryOp / FeatureDiscretizer：业务路径零静默兜底；未注入 `assert`。
2. `BackendVisualRegistry`：实例 + `setProcessInstance`，组合根注入；静态 API 转发。
3. **仍另轨**：`BackendDataManager::instance`；`TrajectoryOpConfigRegistry` / `FeatureDiscretizerConfigRegistry` 配置侧单例。
