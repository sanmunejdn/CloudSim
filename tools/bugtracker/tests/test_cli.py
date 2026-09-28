"""T11 CLI：子命令往返、JSON 输出、退出码。"""
from __future__ import annotations

import json

from bugtracker.cli import main as cli_main


def _run(capsys, *argv) -> tuple[int, dict]:
    code = cli_main(list(argv))
    out = capsys.readouterr().out
    return code, json.loads(out)


def test_create_list_show(client, capsys):
    code, created = _run(capsys, "create", "--title", "CLI 提单",
                         "--severity", "fatal", "--priority", "P0")
    assert code == 0
    bug_id = created["id"]

    code, listing = _run(capsys, "list")
    assert code == 0
    assert any(b["id"] == bug_id for b in listing["items"])

    code, detail = _run(capsys, "show", str(bug_id))
    assert code == 0
    assert detail["title"] == "CLI 提单"
    assert detail["reporter_name"] == "agent"


def test_status_flow_and_invalid(client, capsys):
    code, created = _run(capsys, "create", "--title", "状态测试")
    bug_id = created["id"]
    # new → fixed 非法：退出码 3 + 错误 JSON
    code, err = _run(capsys, "status", str(bug_id), "fixed")
    assert code == 3
    assert err["status_code"] == 409
    # 合法流转
    code, _ = _run(capsys, "status", str(bug_id), "confirmed")
    assert code == 0
    code, r = _run(capsys, "status", str(bug_id), "in_progress",
                   "--comment", "开始处理")
    assert code == 0
    assert r["status"] == "in_progress"


def test_update_and_comment(client, capsys):
    code, created = _run(capsys, "create", "--title", "更新测试")
    bug_id = created["id"]
    code, r = _run(capsys, "update", str(bug_id), "--priority", "P0",
                   "--assignee", "admin")
    assert code == 0
    assert set(r["changed"]) == {"priority", "assignee_id"}
    code, r = _run(capsys, "comment", str(bug_id), "CLI 评论 @admin")
    assert code == 0
    code, detail = _run(capsys, "show", str(bug_id))
    assert detail["assignee_name"] == "admin"
    assert any("CLI 评论" in c["body"] for c in detail["comments"])


def test_search_stats_projects(client, capsys):
    _run(capsys, "create", "--title", "搜索目标-布尔崩溃")
    code, r = _run(capsys, "search", "布尔崩溃")
    assert code == 0 and r["total"] == 1
    code, r = _run(capsys, "stats")
    assert code == 0 and r["total"] >= 1
    code, r = _run(capsys, "projects")
    assert code == 0 and any(p["key"] == "CLOUDSIM" for p in r)


def test_show_missing_bug_exit_3(client, capsys):
    code, err = _run(capsys, "show", "9999")
    assert code == 3
    assert err["status_code"] == 404


def test_export_file(client, capsys, tmp_path):
    _run(capsys, "create", "--title", "导出测试")
    out = tmp_path / "dump.json"
    code, r = _run(capsys, "export", "--output", str(out))
    assert code == 0
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["tables"]["bugs"]
