# FINAL — 资源路径统一

## 结论

按方案 **A + models 构建同步 + 删除 bin 副本** 落地。开发只需记住：模块旁 `resource` 改文本资源，`bin\resource\models` 改模型，OutDir 运行。

## 已做变更

1. `CloudSim\resource` → `CloudSim\src\UI\RobotWidget\resource\`（ControllerPython、controllers）
2. `RobotWidget.vcxproj` 拷贝源改为 `$(ProjectDir)resource\...`
3. `CloudSim.vcxproj` 新增 `SyncSharedModels`（robocopy `bin\resource\models` → OutDir）
4. `ExternalControllerDialog` 开发态回退改 RobotWidget 源路径
5. 删除 `bin\resource\Python`、`bin\resource\trajectory`
6. 打包 `collect-deps.ps1`：SharedResource 仅 models；Desktop 增加 Controller/controllers 源 fallback
7. 相关 README / 验收路径更新

## 验证

- Debug|x64、Release|x64：`RobotWidget` + `CloudSim` 构建成功
- `bin\x64d\resource` 与 `bin\x64\resource` 均含 ControllerPython、controllers、ExportPython、models
