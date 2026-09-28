# 10 — Bug 管理（BugTracker）

**开发新功能、修复 bug 前必须阅读本章。** 本仓库所有缺陷统一在 BugTracker 跟踪，
禁止只在对话/聊天/口头中记录。

实现与详细设计：[../features/Bug管理系统/](../features/Bug管理系统/) ·
使用踩坑记录：[../../tools/bugtracker/README.md](../../tools/bugtracker/README.md)

## 服务管理

```powershell
# 一键后台启动 / 停止（进程脱离终端存活，任意目录可执行，无需 cd）
powershell -ExecutionPolicy Bypass -File "<仓库根>\CloudSim\tools\bugtracker\start.ps1"
powershell -ExecutionPolicy Bypass -File "<仓库根>\CloudSim\tools\bugtracker\stop.ps1"
```

| 项 | 说明 |
|----|------|
| Web 入口 | <http://127.0.0.1:8500>（局域网 `http://<本机IP>:8500`） |
| 前台调试 | `run.ps1`（日志直出控制台，Ctrl+C 停止） |
| 状态判断 | `data\server.pid` 存在 + 浏览器能登录 = 运行中 |
| 数据位置 | `tools/bugtracker/data/`（SQLite + 附件，已 gitignore） |

## 角色与状态机

| 角色 | 能力 |
|------|------|
| admin | 全部 + 用户/项目/模块/标签管理 + 导入导出 |
| dev | 确认/指派/修复流转、评论、附件、关联 commit |
| tester | 提单、回归验证/关闭/重开 |
| viewer | 只读 |

状态机：`新建→已确认→处理中→已修复→回归验证→已关闭`，支持重开/拒绝/重复/挂起；
非法流转返回 409 及允许的目标状态，按提示操作，不要强行重试。

## 修 bug 标准流程（强制）

1. **查单**：确认上下文（MCP `bt_get_bug` / Web 详情页）
2. **认领**：状态置 `in_progress`
3. **修复**：C++ 改动按 [03 构建约定](03-构建约定.md) 双配置编译验证
4. **提交**：commit message 写 `fix #ID`（hook 自动置「已修复」并关联 commit）
5. **闭环**：置 `fixed` + 评论说明修复方案与回归用例
6. **回归**：tester 验证 → `verified` → `closed`；不通过则 `reopened`

## 开发新功能时的纪律（强制）

- 开发中发现的缺陷 → **立即提单**（即使是自己刚写的代码）
- 发现与当前任务**无关**的疑似缺陷 → 提单记录现象与位置，**不擅自修复**
- commit message 引用 `#ID`：`fix #ID` 关闭，`ref #ID` 仅关联

## Agent 通道（AI 协作）

Agent 必须遵守仓库根 `.cursor/rules/bugtracker.mdc`（alwaysApply 强制加载）。

1. **MCP（首选）**：根 `.cursor/mcp.json` 已注册，10 个 `bt_*` 工具
   （查询/提单/流转/评论/统计）。直连 SQLite，Web 服务停止也能用。
2. **CLI（降级）**：工作目录 `CloudSim/tools/bugtracker/`，JSON 输出，退出码 0/2/3

   ```powershell
   python -m bugtracker.cli list --status new,in_progress
   python -m bugtracker.cli create --title "导入崩溃" --severity fatal --priority P0
   python -m bugtracker.cli status 42 fixed --comment "已修复"
   ```

Agent 写操作归属内置用户 `agent`，Web 端审计历史可见。

## git / CI 联动

```powershell
# 目标仓库安装 hook（每仓库一次）
<仓库根>\CloudSim\tools\bugtracker\scripts\install_hook.ps1 -RepoPath <目标仓库路径>
# CI 失败自动建 bug 草稿（CloudSim 仓库根执行）
python tools/bugtracker/scripts/ci_failure_to_bug.py
```

注意：状态机不允许跳态时（如 `confirmed` 收到 `fix` 提交），hook 降级为仅关联 commit，需先人工流转到 `in_progress`。

## 字段约定

| 字段 | 取值 |
|------|------|
| severity | fatal（崩溃/数据损坏）/ major（功能错误）/ minor（瑕疵）/ suggestion（建议） |
| priority | P0（立即）~ P3（排期） |
| environment | debug / release（注明发现时的构建配置） |

---
← [09 打包·SDK·日志](09-打包SDK与日志.md) · [手册目录](README.md)
