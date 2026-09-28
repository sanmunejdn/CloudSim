# FINAL — Bug 管理系统项目总结

## 1. 交付概述

为 CloudSim 项目交付了一套**完整功能的 Bug 管理系统**（BugTracker），独立 Web 应用 + AI Agent 双通道，2026-09-28 完成全部 13 个原子任务并通过验收。

## 2. 系统架构

```
┌─────────────────────────────────────────────────────┐
│  浏览器 SPA（原生 JS + ECharts，深色主题）            │
├─────────────────────────────────────────────────────┤
│  FastAPI REST API（/api/*，会话 Cookie 认证）         │
│  ├─ auth/users/projects/bugs/comments/attachments   │
│  ├─ stats/notifications/filters/integrations/admin  │
│  └─ services: workflow 状态机 / notify / gitlink    │
│               / export / bug_ops / agent_ops        │
├─────────────────────────────────────────────────────┤
│  SQLAlchemy + SQLite（WAL，16 表）                   │
└─────────────────────────────────────────────────────┘
        ▲                              ▲
  git post-commit hook          AI Agent（MCP stdio / CLI）
  ci_failure_to_bug.py          直连 SQLite，复用同一状态机
```

## 3. 功能清单（F1~F15 全部交付）

| # | 功能 | # | 功能 |
|---|---|---|---|
| F1 | 用户认证/四角色权限 | F9 | 统计看板（4 图） |
| F2 | 项目/模块树/里程碑 | F10 | 站内通知 |
| F3 | Bug 全字段 CRUD | F11 | 保存过滤器 |
| F4 | 状态机 + 审计日志 | F12 | JSON/CSV 导入导出 |
| F5 | 评论/附件/关注 | F13 | git commit 联动 |
| F6 | 多维度检索/分页 | F14 | CI 失败自动建 bug |
| F7 | 标签系统 | F15 | **Agent 通道（MCP+CLI+Cursor 规则）** |
| F8 | 严重度×优先级矩阵 | | |

## 4. 关键设计决策

1. **技术栈**：Python 3.12 + FastAPI + SQLAlchemy + SQLite，零外部服务依赖，单 `run.ps1` 启动
2. **Agent 通道直连 SQLite**：MCP Server 不依赖 Web 进程，Agent 在 Web 服务未启动时仍可读写；写操作归属种子用户 `agent`，复用同一状态机与审计，保证数据一致性
3. **状态机严格化**：git 联动在状态机不允许时降级为仅关联（如 confirmed 状态收到 fix 提交），不强行跳态，保证流程可追溯
4. **四角色等级制**：admin > dev > tester > viewer，集成令牌独立于会话体系

## 5. 质量指标

- **测试**：90 个 pytest 全绿，覆盖全部 API/服务/Agent 通道
- **E2E**：浏览器 8 大页面全验证；git hook / CI 脚本真实环境实测
- **代码**：约 5000 行 Python + 1500 行前端，中文注释聚焦 Why
- **隔离性**：未触碰 CloudSim 任何现有 C++/vcxproj 代码

## 6. 文件落点

```
CloudSim/tools/bugtracker/     # 系统本体
  bugtracker/                  # 后端包（api/services/models/mcp_server/cli）
    static/                    # 前端 SPA（包内）
  scripts/                     # git hook / CI 脚本
  tests/                       # 90 个 pytest
  run.ps1 / README.md / .env.example / requirements.txt
.cursor/mcp.json               # MCP 注册（仓库根）
.cursor/rules/bugtracker.mdc   # Agent 使用规则（仓库根）
CloudSim/docs/features/Bug管理系统/  # 6A 全流程文档
```

## 7. 使用方式

```powershell
# 一键后台启动/停止（进程脱离终端存活，任意目录可执行）
CloudSim/tools/bugtracker/start.ps1
CloudSim/tools/bugtracker/stop.ps1
# 前台调试：run.ps1（Ctrl+C 停止）
# 浏览器打开 http://127.0.0.1:8500 ，admin / admin123（首登强制改密）
```

使用踩坑记录（cmd 跨盘符、执行策略、状态判断等）见 README「使用注意事项」。

AI Agent 通过 MCP `bt_*` 工具或 `python -m bugtracker.cli` 直接读写，详见 README。
