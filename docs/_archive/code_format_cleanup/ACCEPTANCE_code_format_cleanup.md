# ACCEPTANCE — CloudSim 源码格式与筛选器整理

日期：2026-07-17

## 范围

- 自研 `src/**` 产品源码（约 968 个 `.h/.cpp`）
- 排除 ThirdParty / vcglib / InstantMeshes 第三方树 / bin / moc 生成物

## 已完成项

| 项 | 结果 |
|----|------|
| `.clang-format`（Tab + Allman + IncludeCategories，兼容 clang-format 12） | 是 |
| UTF-8 with BOM + CRLF | 968 文件已规范化 |
| `.gitattributes` `eol=crlf` | 是 |
| 中文乱码修复 | 4 文件语义修复；复扫 `still_flagged=0` |
| Include 守卫 `工程名_文件名_H` | 525 个头文件 |
| 缺失 `/// @file` / `/// @brief` | 684 文件补齐 |
| clang-format 批量排版 | 968 文件，`batch_failures=0` |
| `.vcxproj.filters` 仅补缺 | 10 个工程新建 filters |
| 约定文档更新 | `SOURCE_CONVENTIONS.md`、`MODULE_DEVELOPER_GUIDES.md`、`DIRECTORY_LAYOUT.md`、docs/README.md、`docs/README.md`、cpp/architecture cursor rules |

## 乱码修复清单

- `src/UI/Widget/inc/widget_global.h` — `HPL_CATCH` 字符串 →「空指针、野指针：」
- `src/Host/CloudSimHost/inc/osg/QWidgetViewer.h` — 文件头与注释重写
- `src/Host/CloudSimHost/inc/osg/GraphicsWindowQt1.h` — 注释重写
- `src/Host/CloudSimHost/source/osg/GraphicsWindowQt1.cpp` — 析构/释放注释重写

详见 `mojibake_report.txt`（若存在）。

## 新建 filters

- InstantMeshesCore / InstantMeshesLib
- CloudSimLabelingSDK / CloudSimMeshTrajectorySDK
- GeometryPlugin / LabelingPlugin / PointCloudPlugin / PointNetPlugin / HelloAiPlugin
- CloudSimUiAssets

## 维护脚本（复跑）

```bash
# 在 CloudSim 根目录
python scripts/fix_chinese_mojibake.py
python scripts/normalize_header_guards.py
python scripts/ensure_file_headers.py
python scripts/run_clang_format.py
python scripts/normalize_source_encoding.py
python scripts/generate_vcxproj_filters.py --only-missing
```

clang-format 路径默认：VS2019 `VC\Tools\Llvm\x64\bin\clang-format.exe`（v12）。

## 冒烟编译

- `CloudSimCore.vcxproj` Debug|x64 — **通过** → `bin\x64d\CloudSimCore.dll`
- `Data.vcxproj` Debug|x64 — **通过** → `bin\x64d\Data.dll`（仅既有 C4251/C4267 警告）

## 抽查

- `EventHub.h`：`CLOUDSIMCORE_EVENTHUB_H`，BOM+CRLF，`@file`/`@brief`，无 `#pragma once`
- `MeshBoolean.h`：`DATA_MESHBOOLEAN_H`，BOM+CRLF
- `widget_global.h`：中文日志字符串正确，`WIDGET_WIDGET_GLOBAL_H`

## 后续备注

- `@brief` 曾按 48 字截断，已用 `scripts/fix_truncated_briefs.py` 恢复完整句；`ensure_file_headers.py` 已取消截断
- `generate_vcxproj_filters.py` 仅扫描 `src/` 下产品 vcxproj（避免 InstantMeshes 嵌套工程）
