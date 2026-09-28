"""T4 项目 / 模块树 / 里程碑 / 标签。"""
from __future__ import annotations

from conftest import USERS, login, make_bug


def test_default_project_seeded(client, admin):
    projects = client.get("/api/projects").json()
    assert any(p["key"] == "CLOUDSIM" for p in projects)


def test_create_project_and_duplicate_key(client, admin):
    r = client.post("/api/projects", json={"key": "SIM2", "name": "仿真2"})
    assert r.status_code == 201
    r = client.post("/api/projects", json={"key": "SIM2", "name": "重复"})
    assert r.status_code == 409


def test_non_admin_cannot_create_project(client, users):
    login(client, "dev1", USERS["dev1"][0])
    assert client.post("/api/projects",
                       json={"key": "X", "name": "x"}).status_code == 403


def test_module_tree(client, admin):
    r = client.post("/api/projects/1/modules", json={"name": "几何内核"})
    assert r.status_code == 201
    parent_id = r.json()["id"]
    r = client.post("/api/projects/1/modules",
                    json={"name": "布尔运算", "parent_id": parent_id})
    assert r.status_code == 201
    projects = client.get("/api/projects").json()
    tree = [p for p in projects if p["id"] == 1][0]["modules"]
    assert tree[0]["name"] == "几何内核"
    assert tree[0]["children"][0]["name"] == "布尔运算"
    # 有子模块不能删
    assert client.delete(f"/api/modules/{parent_id}").status_code == 409


def test_module_parent_must_same_project(client, admin):
    client.post("/api/projects", json={"key": "P2", "name": "项目2"})
    r = client.post("/api/projects/1/modules", json={"name": "M1"})
    m1 = r.json()["id"]
    r = client.post("/api/projects/2/modules",
                    json={"name": "M2", "parent_id": m1})
    assert r.status_code == 422


def test_module_referenced_cannot_delete(client, users):
    login(client, "tester1", USERS["tester1"][0])
    r = client.post("/api/projects/1/modules", json={"name": "临时模块"})
    # tester 无权限建模块
    assert r.status_code == 403
    login(client, "admin", "admin123")
    r = client.post("/api/projects/1/modules", json={"name": "临时模块"})
    mid = r.json()["id"]
    login(client, "tester1", USERS["tester1"][0])
    make_bug(client, title="引用模块的 bug", module_id=mid)
    login(client, "admin", "admin123")
    assert client.delete(f"/api/modules/{mid}").status_code == 409


def test_milestone_crud(client, admin):
    r = client.post("/api/projects/1/milestones",
                    json={"name": "v1.0", "due_date": "2026-12-31"})
    assert r.status_code == 201
    mid = r.json()["id"]
    r = client.patch(f"/api/milestones/{mid}",
                     json={"name": "v1.0", "due_date": "2026-12-31",
                           "status": "closed"})
    assert r.status_code == 200
    assert client.patch(f"/api/milestones/{mid}", json={
        "name": "v1.0", "status": "bad"}).status_code == 422
    assert client.delete(f"/api/milestones/{mid}").status_code == 200


def test_delete_project_with_bugs_forbidden(client, users):
    login(client, "tester1", USERS["tester1"][0])
    make_bug(client, title="占位 bug")
    login(client, "admin", "admin123")
    assert client.delete("/api/projects/1").status_code == 409


def test_labels_crud(client, admin):
    r = client.post("/api/labels", json={"name": "崩溃", "color": "#ff0000"})
    assert r.status_code == 201
    assert client.post("/api/labels",
                       json={"name": "崩溃"}).status_code == 409
    label_id = r.json()["id"]
    labels = client.get("/api/labels").json()
    assert any(l["name"] == "崩溃" for l in labels)
    assert client.delete(f"/api/labels/{label_id}").status_code == 200
