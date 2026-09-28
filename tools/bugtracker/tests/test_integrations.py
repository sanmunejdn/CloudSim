"""T10 集成：git commit 联动 / CI 失败草稿；gitlink 解析规则。"""
from __future__ import annotations

from conftest import USERS, login, make_bug
from bugtracker.services.gitlink import parse_commit_message

TOKEN = {"X-Integration-Token": "test-token"}


# ---- gitlink 解析 ----

def test_parse_fix_and_ref():
    fix, ref = parse_commit_message("fix #12 修复崩溃，ref #34，另见 #56")
    assert fix == {12}
    assert ref == {34, 56}


def test_parse_keywords():
    for msg in ("fixes #1", "close #2", "closed #3", "resolve #4", "resolved #5"):
        fix, _ = parse_commit_message(msg)
        assert fix, msg


def test_parse_no_ids():
    fix, ref = parse_commit_message("普通提交，没有引用")
    assert fix == set() and ref == set()


# ---- commit 端点 ----

def test_commit_requires_token(client, users):
    assert client.post("/api/integrations/commit", json={
        "hash": "abcdef1", "message": "fix #1"}).status_code == 401
    assert client.post("/api/integrations/commit",
                       headers={"X-Integration-Token": "wrong"},
                       json={"hash": "abcdef1", "message": "fix #1"}
                       ).status_code == 401


def test_commit_fix_transitions_bug(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client, title="将被 commit 修复")
    bid = bug["id"]
    login(client, "dev1", USERS["dev1"][0])
    client.post(f"/api/bugs/{bid}/status", json={"to": "confirmed"})
    client.post(f"/api/bugs/{bid}/status", json={"to": "in_progress"})

    r = client.post("/api/integrations/commit", headers=TOKEN, json={
        "repo": "CloudSim", "hash": "abcdef123456",
        "message": f"fix #{bid} 修复空指针", "author": "dev1"})
    assert r.status_code == 200
    assert r.json()["fixed"] == [bid]

    detail = client.get(f"/api/bugs/{bid}").json()
    assert detail["status"] == "fixed"
    assert detail["resolution"] == "fixed"
    assert detail["commits"][0]["commit_hash"] == "abcdef123456"
    # 审计归属 agent
    history = client.get(f"/api/bugs/{bid}/history").json()
    assert any(h["username"] == "agent" and h["new_value"] == "fixed"
               for h in history)


def test_commit_ref_only_links(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client)
    r = client.post("/api/integrations/commit", headers=TOKEN, json={
        "hash": "bbbbbbb1", "message": f"ref #{bug['id']} 相关改动"})
    assert r.json()["linked"] == [bug["id"]]
    assert r.json()["fixed"] == []
    assert client.get(f"/api/bugs/{bug['id']}").json()["status"] == "new"


def test_commit_missing_bug_reported(client, admin):
    r = client.post("/api/integrations/commit", headers=TOKEN, json={
        "hash": "ccccccc1", "message": "fix #9999"})
    assert r.json()["missing"] == [9999]


def test_commit_fix_on_suspended_degrades_to_link(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client)
    bid = bug["id"]
    login(client, "dev1", USERS["dev1"][0])
    client.post(f"/api/bugs/{bid}/status", json={"to": "confirmed"})
    client.post(f"/api/bugs/{bid}/status", json={"to": "in_progress"})
    client.post(f"/api/bugs/{bid}/status", json={"to": "suspended"})
    r = client.post("/api/integrations/commit", headers=TOKEN, json={
        "hash": "ddddddd1", "message": f"fix #{bid}"})
    # suspended 不允许直接 fixed → 降级为仅关联
    assert r.json()["fixed"] == []
    assert r.json()["linked"] == [bid]


# ---- CI 失败草稿 ----

def test_ci_failure_creates_draft(client, admin):
    r = client.post("/api/integrations/ci-failure", headers=TOKEN, json={
        "stamp": "20260928-101500",
        "summary": "run_checks 失败：api 测试 2 例失败",
        "project_key": "CLOUDSIM",
        "failures": [{"name": "test_import", "message": "assert 1 == 2"},
                     {"name": "test_export", "message": "timeout"}],
    })
    assert r.status_code == 201
    bug_id = r.json()["bug_id"]
    detail = client.get(f"/api/bugs/{bug_id}").json()
    assert detail["status"] == "new"
    assert detail["reporter_name"] == "agent"
    assert "[CI]" in detail["title"]
    assert "test_import" in detail["description"]
    assert detail["priority"] == "P1"
