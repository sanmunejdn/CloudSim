# Instant Meshes 源码部署（CloudSim）

上游仓库：[wjakob/instant-meshes](https://github.com/wjakob/instant-meshes)

CloudSim 将 batch 管线编译为静态库 `InstantMeshesLib`，由 `InstantMeshesCore` 进程内调用 `batch_process`（定义 `INSTANT_MESHES_HAS_LIB`）。

## 目录

| 路径 | 说明 |
|------|------|
| `../bin/SDK/instant-meshes/` | 上游源码（与 CloudSim 仓库同级，**不在** CloudSim git 树内） |
| `CloudSim/src/Geometry/InstantMeshesLib/` | VS 工程，仅编入 batch 所需源文件 |
| `CloudSim/src/Geometry/InstantMeshesCore/` | 桥接层 `ImBatchBridge.cpp` |

> CloudSim 的 git 根目录为 `CloudSim/`，无法将 `../bin/SDK` 注册为 git submodule。请用下方脚本或手动 clone，并以 `CLOUDSIM_PINNED_COMMIT` 固定版本。

## 初始化

```powershell
# 在 CGAL5.5.2 根目录执行
.\CloudSim\scripts\init_instant_meshes.ps1
```

或手动：

```bash
git clone https://github.com/wjakob/instant-meshes.git bin/SDK/instant-meshes
cd bin/SDK/instant-meshes
git checkout 7b3160864a2e1025af498c84cfed91cbfb613698
```

## 构建顺序（Visual Studio x64）

1. `InstantMeshesLib`（Debug\|x64 或 Release\|x64）
2. `InstantMeshesCore`（自动链接 `InstantMeshesLib.lib` + OCCT `tbb12.lib`）
3. `GeometryAlgorithm` → 其余解决方案项目

预处理器（InstantMeshesLib / 已启用库的 InstantMeshesCore）：

- `PARALLELIZE`, `SINGLE_PRECISION`, `NOMINMAX`, `_CRT_SECURE_NO_WARNINGS`, `__TBB_NO_IMPLICIT_LINKAGE`
- `INSTANT_MESHES_HAS_LIB`（仅 InstantMeshesCore）

依赖：

- Eigen：`bin/SDK/eigen`
- TBB：bundled `instant-meshes/ext/tbb`（legacy API，**非** OCCT tbb12）。首次需 CMake 构建 `tbb_static.lib`（见 `init_instant_meshes.ps1`）。CloudSim Debug 配置链接 **Release** `tbb_static.lib`（/MD 一致）。

## 运行时

- 主路径：进程内 `batch_process`，无需 `instant-meshes.exe`
- Fallback：未定义 `INSTANT_MESHES_HAS_LIB` 或库调用失败时，仍可通过 `CLOUDSIM_INSTANT_MESHES_EXE` / PATH 调用外部 exe

## Spike 验收（data_2）

对 `bin/SDK/CODE_AMRTO/data_2/meshlab_suitable_catmull.obj` 跑 IM batch，quad 面数应与 `Hole_quad_InstantMeshes_7.87k.obj` 同量级（约 31k 面，±15%）。见 `GeometryAlgorithm` SelfTest `meshSurfaceImRemesh`。
