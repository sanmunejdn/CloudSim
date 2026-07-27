# CloudSim 源码格式约定

> **权威约定**（2026-07 起）。与 `.cursor/rules/cloudsim-cpp-conventions.mdc`、[`MODULE_DEVELOPER_GUIDES.md`](MODULE_DEVELOPER_GUIDES.md) 保持一致。验收记录见 [`code_format_cleanup/ACCEPTANCE_code_format_cleanup.md`](code_format_cleanup/ACCEPTANCE_code_format_cleanup.md)。

适用范围：`src/` 下自研 `*.h` / `*.hpp` / `*.cpp` / `*.cxx` / `*.cc`。  
**排除**：`ThirdParty`、`vcglib`、InstantMeshes 第三方树、`bin/`、moc/uic 生成物。

---

## 1. 文件编码与换行

| 项 | 约定 |
|----|------|
| 字符编码 | **UTF-8 with BOM** |
| 换行 | **CRLF**（`\r\n`） |
| Git | [`.gitattributes`](../.gitattributes) 对 C/C++ 源声明 `text eol=crlf`；BOM 靠脚本维护 |

规范化：

```bash
python scripts/normalize_source_encoding.py
```

须在 clang-format **之后**执行（format 常写出 LF）。

---

## 2. Include 守卫（禁止 `#pragma once`）

统一形式：

```cpp
#ifndef <PROJECT>_<STEM>_H
#define <PROJECT>_<STEM>_H
// ...
#endif // <PROJECT>_<STEM>_H
```

- **`<PROJECT>`**：所属 VS 工程名（`*.vcxproj` 所在目录名）去非字母数字后大写。例：`Data` → `DATA`，`CloudSimCore` → `CLOUDSIMCORE`，`Widget` → `WIDGET`
- **`<STEM>`**：文件名去扩展名，非字母数字改 `_` 后大写。例：`MeshBoolean.h` → `MESHBOOLEAN`，`data_global.h` → `DATA_GLOBAL`
- 示例：`DATA_MESHBOOLEAN_H`、`CLOUDSIMCORE_EVENTHUB_H`、`WIDGET_WIDGET_GLOBAL_H`

批量规范化：

```bash
python scripts/normalize_header_guards.py
```

Host / Widget 若有同名物理副本，各自使用**本工程**前缀（如 `CLOUDSIMHOST_QWIDGETVIEWER_H` vs `WIDGET_QWIDGETVIEWER_H`）。

---

## 3. 文件头注释

每个源文件顶部（头卫之后）应有：

```cpp
/// @file EventHub.h
/// @brief UI 线程事件总线
```

- 中文短句，写职责/Why，不写废话
- 缺失时补齐：`python scripts/ensure_file_headers.py`
- 已有正确中文文件头不改写；乱码文件头在乱码修复步骤中替换

---

## 4. `#include` 顺序与空白排版

仓库根 [`.clang-format`](../.clang-format)（兼容 **clang-format 12**，VS2019 LLVM）：

- Tab 缩进（宽 4）、`BreakBeforeBraces: Allman`、列宽 120
- `SortIncludes` + 分组优先级大致为：
  1. `pch.h` / `*_global.h` / 对应头
  2. C++ 标准库 `<...>`
  3. Qt / OSG / 其它系统头
  4. 项目内 `"..."` 头

批量：

```bash
python scripts/run_clang_format.py
```

默认调用 VS2019：`VC\Tools\Llvm\x64\bin\clang-format.exe`。

模块习惯（与 format 一致）：先本模块 `*_global.h`（若需要），再标准库，再 Qt/OSG，最后项目头。

---

## 5. 中文乱码

检测与修复：

```bash
python scripts/fix_chinese_mojibake.py
# 可选 --dry-run / --report path
```

策略：自动 latin1→gbk 还原 → Host/Widget 对照 → 语义重写注释/可见字符串。不改控制流与标识符。

---

## 6. Visual Studio `.vcxproj.filters`

结构约定（与 [`MODULE_DEVELOPER_GUIDES.md`](MODULE_DEVELOPER_GUIDES.md) 相同）：

1. 顶层：`inc` / `src`（及必要时 `resource`、`ops`、`External`）
2. 子层：**按功能**划分（如 `MainWindow`、`OsgWidget`、`Instructions`、`Simulation`、`adapters` 等）
3. 磁盘已有 `inc\Foo\` / `source\Foo\` / `ops\Name\` 时优先镜像到同名筛选器

| 场景 | 命令 |
|------|------|
| **日常（推荐）** | `python scripts/generate_vcxproj_filters.py --sync` — 保留已有分桶与 GUID，仅为 vcxproj 中**新文件**自动创建功能筛选器 |
| 单工程同步 | `python scripts/generate_vcxproj_filters.py --sync --project RobotWidget` |
| 全量按功能重分类 | `python scripts/generate_vcxproj_filters.py --full`（或 `tools/RegenerateProjectFilters.ps1`） |
| 仅补缺失 filters 文件 | `python scripts/generate_vcxproj_filters.py --only-missing` |

脚本扫描 `CloudSim.sln` 产品工程（外加 PluginHost / HelloAi），写出 **UTF-8 BOM + CRLF**；**不**修改 `.vcxproj` 本体。

**强制**：在 `.vcxproj` 中新增 `ClInclude`/`ClCompile` 后必须跑 `--sync`（Cursor 规则：`cloudsim-vcxproj-filters.mdc`）。

---

## 7. 推荐维护流水线

```text
乱码修复（按需）
  → normalize_header_guards / ensure_file_headers（按需）
  → run_clang_format
  → normalize_source_encoding
  → generate_vcxproj_filters --sync（新增源文件后必做）
```

---

## 8. 与历史文档的关系

- 旧文若仍写「优先 `#pragma once`」或「UTF-8 无 BOM」：**以本文为准**
- 任务归档目录不强制逐份回改；新开发与 Code Review 按本文与 cpp conventions 规则检查
