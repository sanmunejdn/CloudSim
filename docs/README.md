# CloudSim 文档索引

总架构见仓库根 [`../ARCHITECTURE_SUMMARY.md`](../ARCHITECTURE_SUMMARY.md)。本目录按主题存放设计/任务/验收文档；**日常开发约定以本节「常读」为准**。

## 常读（与当前代码一致）

| 文档 | 说明 |
|------|------|
| [`DIRECTORY_LAYOUT.md`](DIRECTORY_LAYOUT.md) | `src/` 域划分、工程对照、构建输出 |
| [`MODULE_DEVELOPER_GUIDES.md`](MODULE_DEVELOPER_GUIDES.md) | 各模块 `DEVELOPER_GUIDE.md` 索引 + 导出/筛选器/注释入口 |
| [`SOURCE_CONVENTIONS.md`](SOURCE_CONVENTIONS.md) | **源码格式权威约定**：编码、头卫、文件头、clang-format、筛选器脚本 |
| [`spatial_contract_world_pose.md`](spatial_contract_world_pose.md) | 世界坐标 / `worldMatrix` 契约（改空间相关代码前必读） |
| [`后端对象与软件模式/`](后端对象与软件模式/) | 后端类型三键、侧车键、工作区模式 vs Data（P0 契约） |

Cursor 规则镜像：`.cursor/rules/cloudsim-cpp-conventions.mdc`、`cloudsim-architecture.mdc`。

## 专题任务文档（历史归档，以代码为准）

| 目录/文件 | 主题 |
|-----------|------|
| [`后端对象与软件模式/`](后端对象与软件模式/) | 后端派生 / 工作区模式边界（P0 已验收文档） |
| [`backend_visibility/`](backend_visibility/) | 后端对象显示/隐藏真源 |
| [`后端对象显示树/`](后端对象显示树/) | Units 多文档显示树（C+B / DisplayForest；**D2 方案待审批后编码**） |
| [`架构边界收口/`](架构边界收口/) | Host/契约边界收口（Sprint A/B/C 验收） |
| [`外部轴联动求解/`](外部轴联动求解/) | 地轨配置 / P0·P_eff / 联立 IK / Run 外轴插帧 |
| [`机器人指令执行/`](机器人指令执行/) | Run 懒规划、轨迹插帧、失败前缀播放 |
| [`backend_persistence/`](backend_persistence/) | 工程持久化 v4 |
| [`code_format_cleanup/`](code_format_cleanup/) | 2026-07 源码格式与筛选器整理验收 |
| [`ui_layout_style_fix/`](ui_layout_style_fix/) | UI 布局与样式 |
| [`离散网格密度控制/`](离散网格密度控制/) | 离散网格密度 |
| [`非刚性配准轨迹算子/`](非刚性配准轨迹算子/) | 非刚性配准轨迹算子 |
| [`vcglib_integration/`](vcglib_integration/) | vcglib / VcgAlgorithms |
| [`mesh_reconstruction_optimization/`](mesh_reconstruction_optimization/) | 曲面重构优化 |
| [`task_brep_refactor_optimization/`](task_brep_refactor_optimization/) | B-rep 重构 |
| [`trajectory_feature_ai.md`](trajectory_feature_ai.md) | AI 轨迹特征 |
| [`spare_nonrigid_registration.md`](spare_nonrigid_registration.md) | SPARE 非刚性配准 |
| [`sdf_nonrigid_registration.md`](sdf_nonrigid_registration.md) | SDF/DDF 混合非刚性配准 |
| [`instant_meshes_build.md`](instant_meshes_build.md) | Instant Meshes 构建 |
| [`mesh_surface_reconstruction.md`](mesh_surface_reconstruction.md) | 网格曲面重构说明 |
| [`UI_Redesign_Plan_CN.md`](UI_Redesign_Plan_CN.md) / [`UI_Redesign_Plan.md`](UI_Redesign_Plan.md) | UI 改版计划 |
| [`widget_bypass_audit.md`](widget_bypass_audit.md) | Widget 绕过 Host 审计 |
| [`algorithm_optimization_summary.md`](algorithm_optimization_summary.md) | 算法优化摘要 |
| [`template_brep_pointcloud_update.md`](template_brep_pointcloud_update.md) | 模板 B-rep / 点云更新 |
| [`工业相机插件/`](工业相机插件/) | 海康一期工业相机 SDK + 手眼标定插件 |
| [`工业相机插件二期/`](工业相机插件二期/) | 梅卡 Mech-Eye + OpenCV 板检测 + 官方手眼候选 |
| [`碰撞检测/`](碰撞检测/) | 网格碰撞、Dock 开关、规划抽样 |

历史任务文档中的接口描述若与现网代码冲突，**以 `src/**/DEVELOPER_GUIDE.md` 与常读文档为准**，不必回溯改写全部归档文。

## 源码格式维护命令（摘要）

在 `CloudSim/` 根目录：

```bash
python scripts/run_clang_format.py
python scripts/normalize_source_encoding.py
python scripts/generate_vcxproj_filters.py --only-missing
```

完整约定与脚本列表见 [`SOURCE_CONVENTIONS.md`](SOURCE_CONVENTIONS.md)。
