"""T6 通知：指派/状态变更/已读管理。"""
from __future__ import annotations

from conftest import USERS, login, make_bug


def _uid(client, username) -> int:
    return [u for u in client.get("/api/users").json()
            if u["username"] == username][0]["id"]


def test_assignee_notified_on_create(client, users):
    login(client, "tester1", USERS["tester1"][0])
    make_bug(client, title="指派通知", assignee_id=_uid(client, "dev1"))
    login(client, "dev1", USERS["dev1"][0])
    notifs = client.get("/api/notifications").json()["items"]
    assert any("创建" in n["message"] for n in notifs)


def test_status_change_notifies_reporter_excludes_actor(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client, assignee_id=_uid(client, "dev1"))
    login(client, "dev1", USERS["dev1"][0])
    client.post("/api/notifications/read-all")
    client.post(f"/api/bugs/{bug['id']}/status", json={"to": "confirmed"})
    # 操作者 dev1 不收自己的通知
    assert client.get("/api/notifications?unread=true").json()["total"] == 0
    # 报告人 tester1 收到
    login(client, "tester1", USERS["tester1"][0])
    notifs = client.get("/api/notifications?unread=true").json()["items"]
    assert any("confirmed" in n["message"] for n in notifs)


def test_read_flow(client, users):
    login(client, "tester1", USERS["tester1"][0])
    make_bug(client, assignee_id=_uid(client, "dev1"))
    login(client, "dev1", USERS["dev1"][0])
    assert client.get("/api/notifications/unread-count").json()["count"] >= 1
    nid = client.get("/api/notifications").json()["items"][0]["id"]
    client.post(f"/api/notifications/{nid}/read")
    client.post("/api/notifications/read-all")
    assert client.get("/api/notifications/unread-count").json()["count"] == 0


def test_cannot_read_others_notification(client, users):
    login(client, "tester1", USERS["tester1"][0])
    make_bug(client, assignee_id=_uid(client, "dev1"))
    login(client, "dev1", USERS["dev1"][0])
    nid = client.get("/api/notifications").json()["items"][0]["id"]
    login(client, "tester1", USERS["tester1"][0])
    assert client.post(f"/api/notifications/{nid}/read").status_code == 404
