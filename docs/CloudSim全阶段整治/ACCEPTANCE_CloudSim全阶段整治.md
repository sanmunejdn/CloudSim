# ACCEPTANCE — CloudSim 全阶段整治

## Phase 1 清理

- [x] 根目录构建日志已删
- [x] `docs/**/build_*.log`、`debug-*.log` 已删
- [x] `src/Geometry/bin`、`src/Infra/bin`、`RobotUrdf/x64`、`RobotWidget/x64` 已删
- [x] `.venv`、`__pycache__`、`RobotCommBridge/obj`、onecad logs 已删
- [x] 历史专题 docs 迁入 `docs/_archive/`，常读保留
- [x] `docs/README.md`、`_archive/INDEX.md`、`DIRECTORY_LAYOUT.md`、架构链接已更新
- [x] HelloAiPlugin / Sketch*Verify / PluginHost.vcxproj **保留**
- [x] 一次性 `tools/gen_*.py`（filters/op 骨架）迁入 `tools/_archive/`；保留有文档入口的 `gen_atomic_ops.py`、`gen_trajectory_json.py`
- [x] `patch_*.py` / `update_*_vcxproj.py`（含 `RobotWidget/tools/patch_controller.py`）迁入 `tools/_archive/`

## Phase 2 审计

- [x] `BUG_AUDIT.md` 含 A/B/C/D 四路

## Phase 3 Top3

| 项 | 状态 | 说明 |
|----|------|------|
| T1 拖放/`m_ops` | 已修 | MIME 门禁、同排 return、插入钳制、`selectedOpIndex` 界于 `m_ops`、删空桩、菜单用 `ops().size()` |
| T2 Run/外轴 | 已修 R1/R2/R8 | 启用外轴禁止 DH/legacy 回退；计划无外轴时不擦指令字段；playbackTimer 空指针防护 |
| T2 延后 | 记录 | R3–R7（多样本外轴、示教 clear、万级轴等） |
| T3 Mesh | 已修 | Face/Shape/Polyline 入口拒绝未实现 mode；Types/头注释对齐 |
| T3 catch | 已修 C01–C04 | robotCollision / coordinateFrames / externalAxes / propertyRows 打 RunLogger::warn |

## Phase 4 双编

| 工程 | Debug\|x64 | Release\|x64 |
|------|------------|--------------|
| GeometryAlgorithm | OK | OK |
| RobotScene | OK | OK |
| RobotWidget | OK | OK |
| CloudSimHost | OK | OK |
| Widget | OK | OK |

产物路径：`bin\x64d\` / `bin\x64\`（工程约定）。
