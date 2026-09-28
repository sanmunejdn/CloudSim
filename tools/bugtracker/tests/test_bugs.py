"""T5 Bug CRUD + 审计；T7 检索/分页。"""
from __future__ import annotations

from conftest import USERS, login, make_bug


def test_create_bug_full_fields(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client, title="导入崩溃", severity="fatal", priority="P0",
                   environment="release", version_found="5.5.2")
    assert bug["status"] == "new"
    assert bug["severity"] == "fatal"
    assert bug["reporter_name"] == "tester1"


def test_create_bug_invalid_enum(client, users):
    login(client, "tester1", USERS["tester1"][0])
    r = client.post("/api/bugs", json={"project_id": 1, "title": "x",
                                       "severity": " catastrophic"})
    assert r.status_code == 422


def test_create_bug_bad_project_module(client, users):
    login(client, "tester1", USERS["tester1"][0])
    assert client.post("/api/bugs", json={
        "project_id": 999, "title": "x"}).status_code == 422
    login(client, "admin", "admin123")
    client.post("/api/projects", json={"key": "P2", "name": "项目2"})
    r = client.post("/api/projects/2/modules", json={"name": "其他项目模块"})
    mid = r.json()["id"]
    login(client, "tester1", USERS["tester1"][0])
    assert client.post("/api/bugs", json={
        "project_id": 1, "title": "x", "module_id": mid}).status_code == 422


def test_viewer_cannot_create(client, users):
    login(client, "viewer1", USERS["viewer1"][0])
    assert client.post("/api/bugs", json={
        "project_id": 1, "title": "x"}).status_code == 403


def test_get_404(client, admin):
    assert client.get("/api/bugs/9999").status_code == 404


def test_patch_writes_audit(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client, title="原标题")
    login(client, "dev1", USERS["dev1"][0])
    r = client.patch(f"/api/bugs/{bug['id']}", json={
        "title": "新标题", "priority": "P0", "assignee_id": _uid(client, "dev1")})
    assert r.status_code == 200
    assert set(r.json()["changed"]) == {"title", "priority", "assignee_id"}
    history = client.get(f"/api/bugs/{bug['id']}/history").json()
    fields = {h["field"] for h in history}
    assert {"created", "title", "priority", "assignee_id"} <= fields
    title_log = [h for h in history if h["field"] == "title"][0]
    assert title_log["old_value"] == "原标题"
    assert title_log["new_value"] == "新标题"
    assert title_log["username"] == "dev1"


def test_patch_permission(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client)
    # 报告人（tester）可改
    r = client.patch(f"/api/bugs/{bug['id']}", json={"description": "补充"})
    assert r.status_code == 200
    # viewer 不可改
    login(client, "viewer1", USERS["viewer1"][0])
    assert client.patch(f"/api/bugs/{bug['id']}",
                        json={"title": "越权"}).status_code == 403


def test_labels_update_and_audit(client, users):
    login(client, "admin", "admin123")
    l1 = client.post("/api/labels", json={"name": "L1"}).json()["id"]
    l2 = client.post("/api/labels", json={"name": "L2"}).json()["id"]
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client, label_ids=[l1])
    assert [lb["name"] for lb in bug["labels"]] == ["L1"]
    r = client.patch(f"/api/bugs/{bug['id']}", json={"label_ids": [l1, l2]})
    assert "labels" in r.json()["changed"]
    assert len(r.json()["bug"]["labels"]) == 2


def _uid(client, username) -> int:
    return [u for u in client.get("/api/users").json()
            if u["username"] == username][0]["id"]


# ---- T7 检索 ----

def _seed_bugs(client):
    login(client, "tester1", USERS["tester1"][0])
    make_bug(client, title="启动崩溃", severity="fatal", priority="P0")
    make_bug(client, title="界面错位", severity="minor", priority="P3")
    login(client, "dev1", USERS["dev1"][0])
    make_bug(client, title="布尔运算结果错误", severity="major", priority="P1")


def test_filter_by_status_and_severity(client, users):
    _seed_bugs(client)
    r = client.get("/api/bugs?severity=fatal,minor")
    assert r.json()["total"] == 2
    r = client.get("/api/bugs?status=new&severity=fatal")
    assert r.json()["total"] == 1


def test_keyword_search(client, users):
    _seed_bugs(client)
    r = client.get("/api/bugs?keyword=布尔")
    assert r.json()["total"] == 1
    assert r.json()["items"][0]["title"] == "布尔运算结果错误"
    # 关键词命中描述
    r = client.get("/api/bugs?keyword=描述")
    assert r.json()["total"] == 3


def test_pagination(client, users):
    _seed_bugs(client)
    r = client.get("/api/bugs?page=1&size=2")
    body = r.json()
    assert body["total"] == 3
    assert len(body["items"]) == 2
    r = client.get("/api/bugs?page=2&size=2")
    assert len(r.json()["items"]) == 1
    # 页码边界
    assert client.get("/api/bugs?page=0").status_code == 422


def test_sort(client, users):
    _seed_bugs(client)
    r = client.get("/api/bugs?sort=id&order=asc")
    ids = [b["id"] for b in r.json()["items"]]
    assert ids == sorted(ids)


def test_saved_filters(client, users):
    login(client, "dev1", USERS["dev1"][0])
    r = client.post("/api/filters", json={
        "name": "我的 P0", "query_json": '{"priority":"P0"}'})
    assert r.status_code == 201
    assert client.post("/api/filters", json={
        "name": "我的 P0", "query_json": "{}"}).status_code == 409
    assert client.post("/api/filters", json={
        "name": "坏", "query_json": "not-json"}).status_code == 422
    fid = r.json()["id"]
    assert len(client.get("/api/filters").json()) == 1
    # 他人不可见/不可删
    login(client, "tester1", USERS["tester1"][0])
    assert client.get("/api/filters").json() == []
    assert client.delete(f"/api/filters/{fid}").status_code == 404
