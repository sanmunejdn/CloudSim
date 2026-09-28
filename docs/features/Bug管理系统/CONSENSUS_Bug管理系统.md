# CONSENSUS — Bug 管理系统

> 基于 ALIGNMENT_Bug管理系统.md，用户已确认全部关键决策点。本文档为最终共识。

## 1. 需求描述

构建一个**独立部署、完整功能的 Bug 管理系统**（命名：**BugTracker**），服务于 CloudSim 及本仓库相关产品的缺陷跟踪，支持团队多用户协作，覆盖 bug 从发现到关闭的全生命周期，并与现有 git 流程、自动化测试体系联动。

## 2. 已确认决策

| 决策点 | 结论 |
|--------|------|
| 系统形态 | **独立 Web 应用**：FastAPI 后端 + SQLite 存储 + 内置 Web 界面（无构建步骤的原生 JS SPA），单命令启动，浏览器访问 |
| 使用范围 | **局域网多用户**：完整账号体系 + 角色权限（管理员 / 开发 / 测试 / 只读） |
| 技术栈 | **Python 3.12 + FastAPI + SQLAlchemy + SQLite**（与现有 scripts/pytest 同栈） |
| 扩展功能 | **全部纳入**：git commit 关联、CI 失败联动、站内通知、项目/模块树 + 里程碑、导入导出 |
| 代码落点 | `CloudSim/tools/bugtracker/`（随 CloudSim 仓库版本管理） |
| 数据落点 | `CloudSim/tools/bugtracker/data/`（SQLite 单文件 + 附件目录，**gitignore**，不入库） |
| 默认端口 | **8500**（可用 `.env` 覆盖） |
| 文档落点 | `CloudSim/docs/features/Bug管理系统/` |

## 3. 功能范围（验收标准锚点）

### 3.1 核心功能

| # | 功能 | 验收标准 |
|---|------|----------|
| F1 | 用户与权限 | 注册需管理员审批或管理员直接建号；四种角色（admin/dev/tester/viewer）权限矩阵生效；登录会话过期机制；密码 PBKDF2 加盐哈希存储 |
| F2 | 项目/模块/里程碑 | 项目 CRUD；模块树（多级）CRUD；里程碑 CRUD（含截止日期、状态） |
| F3 | Bug 全生命周期 | 状态机：新建→已确认→处理中→已修复→回归验证→已关闭；支持重新打开/拒绝/重复/挂起；非法流转被拒绝并提示 |
| F4 | 字段体系 | 标题、描述、复现步骤、期望/实际、严重级别（致命/严重/一般/建议）、优先级（P0~P3）、项目、模块、里程碑、环境（Debug/Release）、发现/修复版本、报告人、指派人、标签 |
| F5 | 评论与审计 | 评论流（支持 @提及触发通知）；所有字段变更自动记录审计历史并可查看 |
| F6 | 附件 | 上传/下载/删除（截图、日志、dump、bundle zip）；大小与类型白名单限制；存储于 `data/attachments/` |
| F7 | 检索 | 多条件组合筛选 + 关键词全文搜索 + 分页 + 排序；保存个人过滤器 |
| F8 | 统计看板 | 状态分布饼图、按模块柱状图、按人员工作量、近 30 天新建/关闭趋势线 |
| F9 | REST API | 全部功能有对应 API；`/api/docs` 自动生成 Swagger 文档 |

### 3.2 扩展功能

| # | 功能 | 验收标准 |
|---|------|----------|
| F10 | git 关联 | commit message 含 `fix #ID` 自动将 bug 置为「已修复」并记录 commit 链接；`ref #ID` 仅关联；提供 post-commit hook 安装脚本 |
| F11 | CI 联动 | 脚本读取 `artifacts/checks/<stamp>/` junit 失败结果 → 调 API 生成 bug 草稿（状态=新建，附失败摘要）；bug 可登记关联回归用例路径 |
| F12 | 通知 | 指派/评论/@提及/状态变更产生站内通知；通知中心可查看、标记已读；邮件通道预留 SMTP 配置接口（`.env`，本期标记 TODO 待真实 SMTP） |
| F13 | 导入导出 | 全量 JSON 导出/导入（含附件清单）；bug 列表 CSV 导出/导入 |
| F14 | 关注机制 | 用户可关注 bug；关注人收到变更通知 |
| F15 | **AI Agent 集成（重点）** | ① **MCP Server**（stdio，注册到根 `.cursor/mcp.json`）：Agent 原生调用查询/新建/流转/评论/统计等工具；**直连 SQLite**（共享 models），Web 服务未启动也能读写；② **CLI**：`python -m bugtracker.cli`，JSON 输出，供 shell/脚本/Agent 备用通道；③ **Cursor 规则** `.cursor/rules/bugtracker.mdc`：规定 Agent 修 bug 前查单、发现 bug 提单、修复后联动状态、commit 引用 `#ID` 的工作纪律；MCP/CLI 写操作归属种子用户 `agent`（可配置） |

## 4. 技术约束与集成方案

- **零构建前端**：原生 HTML/JS/CSS + 本地 vendor ECharts，不引入 npm 构建，保证离线可用、启动即用。
- **安全**：密码 PBKDF2-HMAC-SHA256（stdlib）；会话 Cookie（HttpOnly）；API 鉴权中间件；附件类型白名单；SQL 全部走 ORM 参数化；SMTP 密码等敏感配置仅放 `.env`（gitignore，提供 `.env.example`）。
- **测试**：pytest + FastAPI TestClient，覆盖认证、权限、工作流、CRUD、检索、统计、集成、导入导出；临时库隔离。
- **启动**：`CloudSim/tools/bugtracker/run.ps1` 一键启动（自动建 venv、装依赖、初始化库、首启创建 admin）。
- **与现有体系衔接**：不改动 CloudSim 任何 C++ 工程与构建；仅新增 `tools/bugtracker/` 与文档；git hook 为可选安装。

## 5. 任务边界（明确不做）

- 不做互联网部署、HTTPS 证书、反向代理（局域网内网使用）
- 不做移动端 App / 小程序
- 不做与 GitHub Issues 的双向同步
- 不修改 CloudSim 现有 C++ 源码与构建配置
- 邮件发送仅留接口与配置位，真实 SMTP 凭据列入 TODO

## 6. 验收总标准

1. `run.ps1` 一键启动后，浏览器完成：建用户→建项目/模块→提 bug→流转全状态→评论/附件→检索→看板→导出，全链路可用。
2. pytest 全绿（API 层正常/边界/异常覆盖）。
3. git hook 提交 `fix #1` 后 bug 状态自动变更。
4. CI 脚本对一份 junit 失败样本生成 bug 草稿。
5. **Agent 通道验收**：MCP Server 注册进 `.cursor/mcp.json` 后，新会话 Agent 能直接调用 bug 查询/新建/流转工具；CLI `python -m bugtracker.cli list --json` 输出合法 JSON；`.cursor/rules/bugtracker.mdc` 生效。
6. 文档（README + 本目录 6A 文档）与实现一致。
