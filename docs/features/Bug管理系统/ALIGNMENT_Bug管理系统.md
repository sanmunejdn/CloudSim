# ALIGNMENT — Bug 管理系统

## 1. 原始需求

> 创建一个管理系统管理 bug，要完整的功能实现，不要轻量化。

诉求解读：不满足于「`BUGS.md` 台账」级别的轻量方案，要求一个**具备完整功能、可实际投入使用**的 Bug 管理系统（独立应用，含界面、存储、流程）。

## 2. 项目上下文分析

### 2.1 现有技术栈与资产

| 资产 | 位置 | 与本任务的关系 |
|------|------|----------------|
| C++ 桌面/网页宿主 | `CloudSim/src`、`CloudSim.sln` / `CloudSimWeb.sln` | Bug 的来源产品；可作为集成对象 |
| Web 前端工程 | `CloudSim/web/cloudsim-web-ui`（Vite + TypeScript） | 若集成进 CloudSimWeb 可复用；若独立系统则另建 |
| Python 工具链 | `CloudSim/scripts`、`scripts/`（pytest 已在使用） | 独立系统后端的最顺技术栈 |
| 自动化测试闭环 | `CloudSim/docs/features/自动化测试/BUGFIX_LOOP.md`、`artifacts/` | 失败 bundle / junit / dump 可作为 Bug 附件来源 |
| 文档约定 | `CloudSim/docs/features/<功能名>/` | 本任务文档落点 |

### 2.2 现有痛点（要解决的问题）

- 目前 bug 记录分散：聊天、口头、`BUGFIX_LOOP.md` 只覆盖 CI 失败闭环，**没有统一的 bug 登记、流转、统计入口**。
- 无法回答：还有多少未修？谁在处理？哪个模块 bug 最多？这个 bug 哪个版本修的？

## 3. 需求理解（功能边界草案）

「完整功能」按行业通用 Bug 管理系统理解，候选功能集：

### 核心（必选）

1. **Bug 全生命周期**：新建 → 确认 → 处理中 → 已修复 → 回归验证 → 关闭 / 重新打开；支持拒绝、重复、挂起
2. **字段体系**：标题、描述、复现步骤、期望/实际、严重级别（致命/严重/一般/建议）、优先级、模块归属、版本号、环境（Debug/Release）
3. **人员与指派**：用户账号、指派人、报告人、关注人
4. **评论与历史**：每条 bug 的评论流 + 字段变更审计日志
5. **附件**：截图、日志、crash dump、失败 bundle 上传下载
6. **检索**：多条件筛选、全文搜索、保存过滤器
7. **统计看板**：按状态/模块/人员/时间的分布与趋势图
8. **API**：REST API，便于脚本/CI 自动提单

### 扩展（待确认是否纳入）

9. 与 git commit 关联（`fix #ID` 自动联动状态）
10. 与自动化测试联动（CI 失败自动生成 bug 草稿、关联回归用例）
11. 邮件/通知提醒
12. 项目/模块树管理、里程碑
13. 导入导出（CSV/JSON 备份迁移）

## 4. 关键歧义与决策点（需用户确认）

| # | 决策点 | 候选 |
|---|--------|------|
| Q1 | **系统形态** | A. 独立 Web 应用（推荐：Python FastAPI + SQLite + 内置 Web UI，零依赖可本机/局域网运行）<br>B. 集成进 CloudSimWeb（C++ 宿主内嵌页面，与仿真器同进程）<br>C. 桌面工具（C++/Qt 或 C# WPF） |
| Q2 | **使用范围** | A. 本机单人（免登录或单账号）<br>B. 局域网多用户（账号 + 角色权限：管理员/开发/测试/只读） |
| Q3 | **技术栈** | A. Python（FastAPI，与现有 scripts/pytest 同栈，推荐）<br>B. Node.js（Express/Nest + 前端同栈 TS）<br>C. C# ASP.NET（Windows 生态） |
| Q4 | **扩展功能范围** | 9~13 哪些纳入本期？ |
| Q5 | **部署与数据落点** | 建议：仓库内 `tools/bugtracker/`，数据 `tools/bugtracker/data/bugtracker.db`（SQLite 单文件，随仓库备份）；端口默认 8500 |

## 5. 初步技术建议（待 Q1~Q3 确认后定稿）

- 独立 Web 应用形态下：**FastAPI + SQLite + 服务端渲染/轻量前端单页**，单命令启动 `python -m bugtracker`，浏览器打开即用。
- 测试策略：pytest 覆盖 API 层（正常/边界/异常），遵循「测试优先」。
- 与现有体系衔接：预留 `artifacts/failures/bundle_*.zip` 作为附件来源；commit 关联用 `fix #ID` 约定。

## 6. 疑问澄清记录

| 决策点 | 用户结论 |
|--------|----------|
| Q1 系统形态 | 独立 Web 应用（FastAPI + SQLite + 内置 Web UI） |
| Q2 使用范围 | 局域网多用户（账号 + 四角色权限） |
| Q3 技术栈 | Python FastAPI |
| Q4 扩展功能 | 全部纳入（git 关联 / CI 联动 / 通知 / 项目模块里程碑 / 导入导出） |
| Q6（审批时追加） | **重点：让 AI Agent 在今后的开发中能读取/操作 bug 管理** → 新增 F15：MCP Server（注册进 `.cursor/mcp.json`，Agent 原生工具调用）+ CLI（JSON 输出，shell 可调用）+ Cursor 规则文件（定义 Agent 查 bug/提 bug/改状态的工作纪律）。MCP/CLI 直连 SQLite（共享 models），Web 服务未启动时 Agent 仍可读写 |
