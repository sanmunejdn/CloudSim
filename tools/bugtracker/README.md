# BugTracker — CloudSim 缺陷跟踪系统

独立 Web 应用：FastAPI + SQLite + 内置 SPA（无构建）。覆盖 bug 全生命周期管理、
多用户角色权限、评论/附件/通知、统计看板、git/CI 联动、JSON/CSV 导入导出，
并为 AI Agent 提供 **MCP 工具** 与 **CLI** 两条原生通道。

## 快速开始

```powershell
cd CloudSim/tools/bugtracker
.\start.ps1        # 一键后台启动（首次自动建 venv、装依赖、生成 .env）
.\stop.ps1         # 一键停止
```

`start.ps1` 启动的进程**独立于终端存活**：关闭 Cursor、关闭终端窗口均不影响；
注销/重启系统后需重新执行 `start.ps1`。调试可用 `.\run.ps1` 前台模式（日志直出控制台，Ctrl+C 停止）。

浏览器打开 <http://127.0.0.1:8500>（局域网内其他机器用 `http://<本机IP>:8500`）。

### 使用注意事项（踩坑记录）

1. **启停不依赖当前目录**：脚本内部全是绝对路径，在任意目录直接执行：

   ```powershell
   powershell -ExecutionPolicy Bypass -File "d:\Project\VSprogram\CGAL5.5.2\CloudSim\tools\bugtracker\start.ps1"
   powershell -ExecutionPolicy Bypass -File "d:\Project\VSprogram\CGAL5.5.2\CloudSim\tools\bugtracker\stop.ps1"
   ```

2. **cmd 与 PowerShell 的 cd 差异**：cmd 中 `cd d:\路径` 只记录不切盘（静默无效），
   需 `cd /d d:\路径` 或先输入 `d:` 回车；PowerShell 中 `cd d:\路径` 可直接跨盘。
3. **PowerShell 运行当前目录脚本必须加 `.\`**：输入 `stop.ps1` 不会执行，要 `.\stop.ps1`。
4. **双击 .ps1 默认是打开编辑器**，不是运行；用上面的命令行方式。
5. **报执行策略错误时**：加 `-ExecutionPolicy Bypass`（见第 1 条命令），不要全局改策略。
6. **判断服务真实状态**：以 `data\server.pid` 是否存在 + 浏览器能否登录为准；
   停止后浏览器页面可能因缓存还能打开，但登录/刷新会失败。

- 首启控制台打印 `admin` 随机初始密码（仅一次），登录后强制改密。
  也可在 `.env` 预设 `BT_ADMIN_PASSWORD`。
- 数据落在 `data/`（`bugtracker.db` + `attachments/`），**已 gitignore**。

### 备份与日志

| 项 | 说明 |
|----|------|
| 在线备份 | 设置页「创建备份」或 `POST /api/backup`（admin）：sqlite3 backup API 一致拷贝主库 + 附件到 `data/backups/<时间戳>/`（WAL 安全，勿在服务运行时直接拷贝 `.db`） |
| JSON 导出 | 设置页「导出全量 JSON」——不含附件本体，适合迁移/审计 |
| 应用日志 | `data/server.log`：5MB × 3 代自动轮转 |
| 进程 stdout/err | `data/server.out.log` / `server.err.log`：启动前若 >10MB 保留一代 `.1.log` |

## 角色权限

| 角色 | 能力 |
|------|------|
| admin | 全部 + 用户/项目/模块/里程碑/标签管理 + 数据导入导出 |
| dev | bug 确认/指派/修复流转、评论、附件、关联 commit/用例 |
| tester | 提单、回归验证/关闭/重开、评论、附件 |
| viewer | 只读浏览 |

状态机：`新建→已确认→处理中→已修复→回归验证→已关闭`，支持重开/拒绝/重复/挂起；
非法流转返回 409 及允许的目标状态。所有字段变更与流转均留审计历史。

## AI Agent 通道（重点）

Agent 在开发中可直接读写本系统（规则见仓库根 `.cursor/rules/bugtracker.mdc`）：

1. **MCP（首选）**：已注册在根 `.cursor/mcp.json`，新会话自动获得 `bt_*` 工具
   （查询/提单/流转/评论/统计等 10 个）。**直连 SQLite，Web 服务未启动也能用**。
   前置：先跑过一次 `start.ps1`（生成 `.venv`）。仓库移动位置后需更新 mcp.json 中的绝对路径。
2. **CLI（降级）**：

   ```powershell
   cd CloudSim/tools/bugtracker
   python -m bugtracker.cli list --status new,in_progress
   python -m bugtracker.cli create --title "导入崩溃" --severity fatal --priority P0
   python -m bugtracker.cli status 42 fixed --comment "已修复"
   python -m bugtracker.cli show 42
   ```

   默认 JSON 输出；退出码 0 成功 / 2 参数错 / 3 业务错。

Agent 写操作归属内置用户 `agent`（Web 端审计历史可见）。

## git commit 联动

```powershell
# 在要联动的仓库上安装 hook（只需一次）
.\scripts\install_hook.ps1 -RepoPath d:\Project\VSprogram\CGAL5.5.2\CloudSim
```

之后 commit message 中：

- `fix #123` / `close #123` / `resolve #123` → 自动把 #123 置为「已修复」并记录关联
- `ref #123` 或裸 `#123` → 仅记录关联

## CI 失败联动

```powershell
# 在 CloudSim 仓库根目录：解析最新 artifacts/checks 批次，失败则生成 bug 草稿
python tools/bugtracker/scripts/ci_failure_to_bug.py
```

草稿单报告人为 `agent`，描述含失败用例清单；修复后应在详情页登记回归用例路径。

## 配置（.env）

见 `.env.example`。关键项：`BT_PORT`、`BT_INTEGRATION_TOKEN`（git/CI 集成令牌，
`run.ps1` 首启自动生成随机值）、`BT_SMTP_*`（邮件通知，**TODO 待真实 SMTP**）。
`.env` 已 gitignore，**禁止提交**。

## 测试

```powershell
cd CloudSim/tools/bugtracker
python -m pytest tests/ -q     # 90+ 用例：认证/权限/状态机/附件/统计/集成/MCP/CLI
```

## 目录

```
bugtracker/            # 后端包（api/ services/ models.py security.py ...）
  mcp_server.py        # MCP stdio 服务（Agent 通道）
  cli.py               # 命令行通道
  static/              # 无构建 SPA（index.html app.js style.css vendor/echarts）
scripts/               # git hook 与 CI 联动脚本
tests/                 # pytest 套件
data/                  # 运行数据（gitignore）
```

API 文档：服务启动后访问 <http://127.0.0.1:8500/api/docs>（Swagger）。

## 离线部署

目标机器无网时：在有网机器执行
`pip download -r requirements.txt -d wheels`，把 `wheels/` 拷到目标机后
`pip install --no-index --find-links wheels -r requirements.txt`（在 `.venv` 中）。
