# TODO — 统一 VS 输出目录

1. **可选全量编译**：对其余工程跑一遍 Debug|x64 + Release|x64。
2. **x86**：本次未动；若仍需要 Win32，可另开任务改为相对 `CloudSimRepoRoot` 的 `bin\x86(d)`。
3. ~~SDK 库路径~~：已统一为 `$(CloudSimRepoRoot)bin\SDK\...`。
4. ~~Include / RobotWidget 资源~~：`AdditionalIncludeDirectories`、qtpropertybrowser `ClCompile`、`RobotWidget` Python 资源复制均已统一；冗余 `SolutionDir..\bin\resource` 二次拷贝已删除（`OutDir` 已是 `CloudSimBinDir`）。
