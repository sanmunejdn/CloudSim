"""T6 评论 + @提及通知 + 关注。"""
from __future__ import annotations

from conftest import USERS, login, make_bug


def test_comment_crud(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client)
    r = client.post(f"/api/bugs/{bug['id']}/comments",
                    json={"body": "补充：仅 Release 复现"})
    assert r.status_code == 201
    cid = r.json()["id"]
    comments = client.get(f"/api/bugs/{bug['id']}/comments").json()
    assert comments[0]["username"] == "tester1"
    # 本人可删
    assert client.delete(
        f"/api/bugs/{bug['id']}/comments/{cid}").status_code == 200


def test_viewer_cannot_comment(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client)
    login(client, "viewer1", USERS["viewer1"][0])
    assert client.post(f"/api/bugs/{bug['id']}/comments",
                       json={"body": "x"}).status_code == 403


def test_delete_others_comment_forbidden(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client)
    cid = client.post(f"/api/bugs/{bug['id']}/comments",
                      json={"body": "tester 的评论"}).json()["id"]
    login(client, "dev1", USERS["dev1"][0])
    assert client.delete(
        f"/api/bugs/{bug['id']}/comments/{cid}").status_code == 403
    # admin 可删
    login(client, "admin", "admin123")
    assert client.delete(
        f"/api/bugs/{bug['id']}/comments/{cid}").status_code == 200


def test_mention_triggers_notification(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client)
    client.post(f"/api/bugs/{bug['id']}/comments",
                json={"body": "@dev1 请看一下这个问题"})
    login(client, "dev1", USERS["dev1"][0])
    notifs = client.get("/api/notifications").json()["items"]
    assert any(n["type"] == "mention" and n["bug_id"] == bug["id"]
               for n in notifs)


def test_watch_and_notify(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client)
    bid = bug["id"]
    # dev1 关注
    login(client, "dev1", USERS["dev1"][0])
    client.post(f"/api/bugs/{bid}/watch")
    # tester1 发评论 → dev1（关注人）收到通知，tester1（操作者）不收
    login(client, "tester1", USERS["tester1"][0])
    client.post(f"/api/bugs/{bid}/comments", json={"body": "进展更新"})
    assert client.get("/api/notifications").json()["total"] == 0
    login(client, "dev1", USERS["dev1"][0])
    notifs = client.get("/api/notifications").json()["items"]
    assert any("评论" in n["message"] for n in notifs)
    # 取消关注后不再收
    client.delete(f"/api/bugs/{bid}/watch")
    client.post("/api/notifications/read-all")
    login(client, "tester1", USERS["tester1"][0])
    client.post(f"/api/bugs/{bid}/comments", json={"body": "再次更新"})
    login(client, "dev1", USERS["dev1"][0])
    assert client.get("/api/notifications?unread=true").json()["total"] == 0
