"""T5 状态机：合法/非法流转、resolution、权限、closed_at。"""
from __future__ import annotations

from conftest import USERS, login, make_bug


def _to(client, bug_id, to, **kwargs):
    return client.post(f"/api/bugs/{bug_id}/status",
                       json={"to": to, **kwargs})


def test_full_lifecycle(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client, title="全生命周期")
    bid = bug["id"]
    login(client, "dev1", USERS["dev1"][0])
    assert _to(client, bid, "confirmed").json()["status"] == "confirmed"
    assert _to(client, bid, "in_progress").json()["status"] == "in_progress"
    r = _to(client, bid, "fixed", comment="已修复，见 commit abc")
    assert r.json()["status"] == "fixed"
    assert r.json()["resolution"] == "fixed"
    login(client, "tester1", USERS["tester1"][0])
    assert _to(client, bid, "verified").json()["status"] == "verified"
    r = _to(client, bid, "closed")
    assert r.json()["status"] == "closed"
    assert r.json()["closed_at"] is not None
    # closed 是终态
    assert _to(client, bid, "reopened").status_code == 409


def test_illegal_transition_returns_allowed(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client)
    login(client, "dev1", USERS["dev1"][0])
    r = _to(client, bug["id"], "fixed")
    assert r.status_code == 409
    detail = r.json()["detail"]
    assert "confirmed" in detail["allowed"]


def test_rejected_requires_resolution(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client)
    login(client, "dev1", USERS["dev1"][0])
    assert _to(client, bug["id"], "rejected").status_code == 422
    r = _to(client, bug["id"], "rejected", resolution="bydesign")
    assert r.json()["status"] == "rejected"
    assert r.json()["resolution"] == "bydesign"


def test_reopen_clears_resolution_and_closed_at(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client)
    bid = bug["id"]
    login(client, "dev1", USERS["dev1"][0])
    _to(client, bid, "confirmed")
    _to(client, bid, "in_progress")
    _to(client, bid, "fixed")
    login(client, "tester1", USERS["tester1"][0])
    r = _to(client, bid, "reopened")
    assert r.json()["status"] == "reopened"
    assert r.json()["resolution"] is None
    # 重新走修复流程
    login(client, "dev1", USERS["dev1"][0])
    assert _to(client, bid, "in_progress").status_code == 200


def test_tester_cannot_confirm(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client)
    r = _to(client, bug["id"], "confirmed")
    assert r.status_code == 403


def test_suspend_and_resume(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client)
    bid = bug["id"]
    login(client, "dev1", USERS["dev1"][0])
    _to(client, bid, "confirmed")
    _to(client, bid, "in_progress")
    assert _to(client, bid, "suspended").json()["status"] == "suspended"
    assert _to(client, bid, "in_progress").json()["status"] == "in_progress"


def test_transition_comment_and_audit(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client)
    bid = bug["id"]
    login(client, "dev1", USERS["dev1"][0])
    _to(client, bid, "confirmed", comment="确认是问题")
    history = client.get(f"/api/bugs/{bid}/history").json()
    assert any(h["field"] == "status" and h["new_value"] == "confirmed"
               for h in history)
    comments = client.get(f"/api/bugs/{bid}/comments").json()
    assert any("确认是问题" in c["body"] for c in comments)
