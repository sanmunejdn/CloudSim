# TODO — Bug 管理系统待办与配置清单

## 1. 需要你决策/配置的事项

### 1.1 邮件通知（可选，当前未启用）
- **现状**：站内通知已可用；SMTP 邮件发送代码已预留但未配置
- **操作**：编辑 `CloudSim/tools/bugtracker/.env`，填入：
  ```
  BT_SMTP_HOST=smtp.你的邮箱.com
  BT_SMTP_PORT=465
  BT_SMTP_USER=xxx
  BT_SMTP_PASSWORD=xxx
  ```
- **不配置的影响**：无，站内通知照常工作

### 1.2 局域网访问（如需团队使用）
- **现状**：默认监听 `127.0.0.1:8500`，仅本机
- **操作**：`.env` 中改 `BT_HOST=0.0.0.0`，防火墙放行 8500 端口
- **建议**：团队使用前先在「用户管理」为每人建账号，停用默认 admin 弱密码

### 1.3 开机自启（可选）
- **现状**：`start.ps1` 启动的进程已脱离终端独立存活（关 Cursor/终端不影响），但**注销/重启系统后需手动重跑**
- **操作**：如需开机自启，用「任务计划程序」建一个登录触发任务，执行
  `powershell -ExecutionPolicy Bypass -File <仓库>\CloudSim\tools\bugtracker\start.ps1`

### 1.4 仓库移动后需同步的路径
- 根 `.cursor/mcp.json` 中是**绝对路径**指向 `CloudSim/tools/bugtracker/.venv/Scripts/python.exe`
- 若整个仓库目录移动，需同步修改该路径，否则 MCP 通道失效（CLI 不受影响）

## 2. 环境相关

### 2.1 离线机器部署（如需）
- 当前 `run.ps1` 首次运行需联网装依赖
- 离线方案：在联网机器执行
  ```
  pip download -r requirements.txt -d wheels
  ```
  将 `wheels/` 拷入离线机器，`pip install --no-index --find-links=wheels -r requirements.txt`

### 2.2 git hook 需逐仓库安装
- hook 不是全局的，每个要联动的仓库执行一次：
  ```powershell
  CloudSim/tools/bugtracker/scripts/install_hook.ps1 -RepoPath <目标仓库路径>
  ```

## 3. 已知限制（设计内，非缺陷）

| 项 | 说明 |
|---|---|
| 附件存储 | 本地 `data/attachments/`，无对象存储；用设置页「创建备份」或 `POST /api/backup`（勿在服务运行时直接拷贝 `.db`，WAL 可能撕裂） |
| 并发 | 已设 `busy_timeout=5s` + MCP/CLI 写重试；小团队（≤20 人）无压力；更大规模需换 PostgreSQL |
| 认证 | 会话 Cookie，无 OAuth/LDAP；如需对接域账号需二次开发 |
| Schema 迁移 | 轻量 `PRAGMA user_version`（见 `bugtracker/migrations.py`）；改表时 bump 版本并补迁移函数 |

## 4. 建议的后续增强（未纳入阶段一）

- 登录限流 / 账号锁定
- Web ↔ MCP 双路径契约测试
- git hook 失败补偿队列
- HTTPS / 收敛监听地址
- 看板自定义图表配置
- bug 订阅的邮件摘要（日报/周报）
- 与 CloudSim 插件崩溃上报自动对接（当前需手动或走 CI 脚本）
