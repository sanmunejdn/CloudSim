"""T3 认证与权限：登录/会话/改密/角色矩阵。"""
from __future__ import annotations

from conftest import ADMIN_PASSWORD, USERS, login


def test_health(client):
    r = client.get("/api/health")
    assert r.status_code == 200
    assert r.json()["ok"] is True


def test_login_success(client):
    user = login(client, "admin", ADMIN_PASSWORD)
    assert user["username"] == "admin"
    assert user["role"] == "admin"
    assert user["must_change_password"] is True


def test_login_wrong_password(client):
    r = client.post("/api/auth/login",
                    json={"username": "admin", "password": "wrong"})
    assert r.status_code == 401


def test_login_unknown_user(client):
    r = client.post("/api/auth/login",
                    json={"username": "nobody", "password": "x"})
    assert r.status_code == 401


def test_me_requires_login(client):
    assert client.get("/api/auth/me").status_code == 401


def test_logout_invalidates_session(client, admin):
    assert client.get("/api/auth/me").status_code == 200
    client.post("/api/auth/logout")
    assert client.get("/api/auth/me").status_code == 401


def test_change_password(client, admin):
    r = client.post("/api/auth/password", json={
        "old_password": "wrong", "new_password": "newpass123"})
    assert r.status_code == 400
    r = client.post("/api/auth/password", json={
        "old_password": ADMIN_PASSWORD, "new_password": "newpass123"})
    assert r.status_code == 200
    # 当前会话保留，must_change_password 已清除
    me = client.get("/api/auth/me").json()
    assert me["must_change_password"] is False
    client.post("/api/auth/logout")
    login(client, "admin", "newpass123")


def test_inactive_user_cannot_login(client, users):
    r = client.post("/api/auth/login",
                    json={"username": "dev1", "password": USERS["dev1"][0]})
    assert r.status_code == 200
    # 切回 admin 停用 dev1
    login(client, "admin", ADMIN_PASSWORD)
    dev1 = [u for u in client.get("/api/users").json()
            if u["username"] == "dev1"][0]
    r = client.patch(f"/api/users/{dev1['id']}", json={"active": False})
    assert r.status_code == 200
    client.post("/api/auth/logout")
    r = client.post("/api/auth/login",
                    json={"username": "dev1", "password": USERS["dev1"][0]})
    assert r.status_code == 403


def test_role_matrix_viewer_forbidden(client, users):
    login(client, "viewer1", USERS["viewer1"][0])
    # viewer 可以读
    assert client.get("/api/bugs").status_code == 200
    # viewer 不能提单 / 建用户 / 建项目
    assert client.post("/api/bugs", json={
        "project_id": 1, "title": "x"}).status_code == 403
    assert client.post("/api/users", json={
        "username": "hack1", "password": "123456"}).status_code == 403
    assert client.post("/api/projects", json={
        "key": "HACK", "name": "hack"}).status_code == 403
