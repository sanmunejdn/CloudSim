# REGRESSION_CHECKLIST_backend_persistence

## 1. 前置条件

- 安装 Visual Studio Build Tools 或 Visual Studio（包含 MSBuild、v142/v143 C++ 工具链）
- 可在 PowerShell 中执行 `msbuild` 或使用绝对路径调用 `MSBuild.exe`
- 使用 `x64 Debug`（与当前项目配置一致）

## 2. 构建回归

## 2.1 编译 Widget 工程

```powershell
msbuild "CloudSim\CloudSim.sln" /t:Widget /p:Configuration=Debug /p:Platform=x64 /m
```

验收：

- 编译通过
- 无新增编译错误（尤其是 `MainWindowProjectIo.cpp`、`BackendDataBase.cpp`）

## 2.2 全量编译（建议）

```powershell
msbuild "CloudSim\CloudSim.sln" /p:Configuration=Debug /p:Platform=x64 /m
```

验收：

- 方案整体编译通过

## 3. 功能回归（保存/恢复）

准备一个包含以下元素的工程：

- 至少 1 个点云对象
- 至少 1 个网格对象
- 至少 1 条父子层级边
- 至少 1 个 follow 组件（子对象跟随父对象）
- 至少 1 组机器人程序（若当前项目已使用机器人模块）

## 3.1 保存验证

步骤：

1. 在 UI 中保存工程（生成 `project.json` 或 `.pcp`）
2. 打开保存结果检查字段

验收：

- 根字段 `version == 4`
- 每个对象包含 `className`
- 对象包含 `geometry`（点云/网格）
- 对象包含 `propertyBag`
- 对象包含 `components`（有 follow 时应含 `type=FollowAttachment`）

## 3.2 恢复验证

步骤：

1. 关闭当前文档
2. 重新打开刚保存的工程
3. 检查树结构、场景和属性面板

验收：

- 对象数量一致
- 对象 id/name 一致
- edges 层级关系一致
- 点云/网格可正常显示
- propertyBag 对应属性值一致
- follow 行为一致（目标对象移动时从对象联动）

## 3.3 兼容验证（旧字段）

步骤：

1. 准备含旧字段 `followAttachment`、无 `components` 的工程样本
2. 打开工程

验收：

- follow 组件仍可恢复（兼容路径生效）

## 4. 失败定位建议

- 若加载失败：先看 `RunInfoPage` warning
- 若对象缺失：检查 `className` 是否在 `BackendRegistryBuiltins` 注册
- 若 follow 失效：检查对象 `components` 中 `FollowAttachment` 数据是否完整
