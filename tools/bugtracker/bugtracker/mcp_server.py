"""MCP stdio 服务：AI Agent 原生 bug 管理通道。

直连 SQLite（共享 models/services），不依赖 Web 服务进程。
注册：仓库根 .cursor/mcp.json。启动：python -m bugtracker.mcp_server
"""
from __future__ import annotations

import json

from fastapi import HTTPException
from mcp.server.mcpserver import MCPServer

from .config import load_settings
from .db import db
from .services import agent_ops

mcp = MCPServer("bugtracker")


def _bootstrap() -> None:
    if db.engine is None:
        db.init(load_settings())


def _call(fn, **kwargs) -> str:
    """统一会话管理与错误格式：业务错误以 {"error": ...} 返回，便于 Agent 理解。"""
    _bootstrap()
    try:
        with db.session() as sess:
            actor = agent_ops.agent_user(sess, db.settings.agent_username)
            result = fn(sess, actor, **kwargs)
            return json.dumps(result, ensure_ascii=False, default=str)
    except HTTPException as exc:
        return json.dumps({"error": exc.detail,
                           "status_code": exc.status_code},
                          ensure_ascii=False)


@mcp.tool()
def bt_list_bugs(status: str = "", severity: str = "", priority: str = "",
                 project_key: str = "", module: str = "", assignee: str = "",
                 reporter: str = "", keyword: str = "",
                 page: int = 1, size: int = 20) -> str:
    """多条件查询 bug 列表（默认排除已关闭 closed）。status/severity/priority
    支持逗号分隔多值，如 status="new,in_progress"。返回 {items, total, page, size}。"""
    return _call(agent_ops.list_bugs, status=status, severity=severity,
                 priority=priority, project_key=project_key, module=module,
                 assignee=assignee, reporter=reporter, keyword=keyword,
                 page=page, size=size)


@mcp.tool()
def bt_get_bug(bug_id: int) -> str:
    """获取 bug 完整详情：全部字段 + 评论 + 审计历史 + 关联 commit/回归用例。"""
    return _call(agent_ops.get_bug, bug_id=bug_id)


@mcp.tool()
def bt_search(keyword: str, size: int = 20) -> str:
    """按关键词全文搜索（命中标题/描述/复现步骤），默认排除已关闭。"""
    return _call(agent_ops.search, keyword=keyword, size=size)


@mcp.tool()
def bt_create_bug(title: str, project_key: str = "", severity: str = "major",
                  priority: str = "P2", description: str = "",
                  repro_steps: str = "", expected: str = "", actual: str = "",
                  environment: str = "debug", version_found: str = "",
                  module: str = "", assignee: str = "") -> str:
    """新建 bug。severity: fatal/major/minor/suggestion；priority: P0~P3；
    environment: debug/release。返回 {"id": 新单号}。"""
    return _call(agent_ops.create_bug, title=title, project_key=project_key,
                 severity=severity, priority=priority, description=description,
                 repro_steps=repro_steps, expected=expected, actual=actual,
                 environment=environment, version_found=version_found,
                 module=module, assignee=assignee)


@mcp.tool()
def bt_update_status(bug_id: int, to: str, resolution: str = "",
                     comment: str = "") -> str:
    """状态流转（走状态机，非法流转会被拒绝）。to: confirmed/in_progress/fixed/
    verified/closed/reopened/rejected/duplicate/suspended；rejected 必须给
    resolution（wontfix/bydesign/cannot_reproduce）。"""
    return _call(agent_ops.update_status, bug_id=bug_id, to=to,
                 resolution=resolution or None, comment=comment)


@mcp.tool()
def bt_update_bug(bug_id: int, title: str = "", severity: str = "",
                  priority: str = "", environment: str = "",
                  description: str = "", repro_steps: str = "",
                  expected: str = "", actual: str = "",
                  version_found: str = "", version_fixed: str = "",
                  assignee: str = "", module: str = "") -> str:
    """修改 bug 字段（只传要改的；写审计并通知相关人）。assignee/module 传用户名/
    模块名；assignee 传 "none" 表示取消指派。"""
    def _none_to_empty(value: str) -> str | None:
        return None if value == "" else value

    return _call(agent_ops.update_bug, bug_id=bug_id,
                 title=_none_to_empty(title), severity=_none_to_empty(severity),
                 priority=_none_to_empty(priority),
                 environment=_none_to_empty(environment),
                 description=_none_to_empty(description),
                 repro_steps=_none_to_empty(repro_steps),
                 expected=_none_to_empty(expected), actual=_none_to_empty(actual),
                 version_found=_none_to_empty(version_found),
                 version_fixed=_none_to_empty(version_fixed),
                 assignee=None if assignee == "" else
                 ("" if assignee == "none" else assignee),
                 module=_none_to_empty(module))


@mcp.tool()
def bt_add_comment(bug_id: int, body: str) -> str:
    """给 bug 添加评论（@用户名 会触发站内通知）。"""
    return _call(agent_ops.add_comment, bug_id=bug_id, body=body)


@mcp.tool()
def bt_stats_overview() -> str:
    """统计总览：按状态/严重级别分布、open/closed/total 计数。"""
    return _call(agent_ops.stats_overview)


@mcp.tool()
def bt_list_projects() -> str:
    """列出全部项目（含 bug 数）。"""
    return _call(agent_ops.list_projects)


@mcp.tool()
def bt_list_modules(project_key: str = "") -> str:
    """列出模块（可按项目 key 过滤）。"""
    return _call(agent_ops.list_modules, project_key=project_key)


def main() -> None:
    mcp.run()  # stdio


if __name__ == "__main__":
    main()
