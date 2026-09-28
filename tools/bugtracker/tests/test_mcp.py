"""T11 MCP：工具函数直调 + 真实 stdio 端到端（initialize/list_tools/call_tool）。"""
from __future__ import annotations

import asyncio
import json
import sys
from pathlib import Path

from bugtracker import mcp_server

PKG_ROOT = Path(__file__).resolve().parent.parent


def test_tools_registered():
    names = {t.name for t in asyncio.run(mcp_server.mcp.list_tools())}
    assert {"bt_list_bugs", "bt_get_bug", "bt_create_bug", "bt_update_status",
            "bt_update_bug", "bt_add_comment", "bt_search",
            "bt_stats_overview", "bt_list_projects", "bt_list_modules"} <= names


def test_tool_call_roundtrip(client):
    """client 夹具已把 db 初始化到临时库；直接调用工具函数验证逻辑。"""
    created = json.loads(mcp_server.bt_create_bug(
        title="MCP 提单", severity="fatal", priority="P0",
        description="通过 MCP 创建"))
    bug_id = created["id"]

    detail = json.loads(mcp_server.bt_get_bug(bug_id=bug_id))
    assert detail["title"] == "MCP 提单"
    assert detail["reporter_name"] == "agent"

    listing = json.loads(mcp_server.bt_list_bugs(severity="fatal"))
    assert any(b["id"] == bug_id for b in listing["items"])

    r = json.loads(mcp_server.bt_update_status(bug_id=bug_id, to="confirmed"))
    assert r["status"] == "confirmed"

    r = json.loads(mcp_server.bt_add_comment(bug_id=bug_id,
                                             body="MCP 评论 @admin"))
    assert r["bug_id"] == bug_id

    stats = json.loads(mcp_server.bt_stats_overview())
    assert stats["total"] >= 1


def test_tool_invalid_transition_returns_error_json(client):
    created = json.loads(mcp_server.bt_create_bug(title="非法流转"))
    err = json.loads(mcp_server.bt_update_status(bug_id=created["id"],
                                                 to="closed"))
    assert err["status_code"] == 409
    assert "allowed" in str(err["error"])


def test_tool_missing_bug(client):
    err = json.loads(mcp_server.bt_get_bug(bug_id=9999))
    assert err["status_code"] == 404


def test_stdio_end_to_end(tmp_path):
    """真实 MCP stdio：子进程跑 server，客户端 initialize + list_tools + call。"""
    from mcp import ClientSession, StdioServerParameters
    from mcp.client.stdio import get_default_environment, stdio_client

    # stdio_client 的 env=None 是最小环境而非继承，必须显式传入
    env = get_default_environment() | {
        "BT_DATA_DIR": str(tmp_path / "mcp_data"),
        "BT_ADMIN_PASSWORD": "admin123",
    }

    async def run():
        server = StdioServerParameters(
            command=sys.executable, args=["-m", "bugtracker.mcp_server"],
            env=env, cwd=str(PKG_ROOT))
        async with stdio_client(server) as (read, write):
            async with ClientSession(read, write) as session:
                await session.initialize()
                tools = await session.list_tools()
                names = {t.name for t in tools.tools}
                assert "bt_create_bug" in names

                r = await session.call_tool(
                    "bt_create_bug",
                    {"title": "stdio 端到端", "severity": "minor"})
                payload = json.loads(r.content[0].text)
                assert payload["id"] >= 1

                r = await session.call_tool("bt_stats_overview", {})
                stats = json.loads(r.content[0].text)
                assert stats["total"] == 1

    asyncio.run(run())
