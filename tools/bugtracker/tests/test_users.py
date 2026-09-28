"""T4 用户管理。"""
from __future__ import annotations

from conftest import USERS, login


def test_admin_create_user(client, admin):
    r = client.post("/api/users", json={
        "username": "newbie", "password": "pass123", "role": "dev",
        "display_name": "新人"})
    assert r.status_code == 201
    assert r.json()["must_change_password"] is True


def test_create_user_duplicate(client, admin):
    r = client.post("/api/users", json={
        "username": "admin", "password": "pass123"})
    assert r.status_code == 409


def test_create_user_invalid_role(client, admin):
    r = client.post("/api/users", json={
        "username": "bad1", "password": "pass123", "role": "superadmin"})
    assert r.status_code == 422


def test_non_admin_cannot_create(client, users):
    login(client, "dev1", USERS["dev1"][0])
    r = client.post("/api/users", json={
        "username": "x1", "password": "pass123"})
    assert r.status_code == 403


def test_list_users_contains_seed(client, admin):
    names = {u["username"] for u in client.get("/api/users").json()}
    assert {"admin", "agent"} <= names


def test_patch_user_role_and_reset_password(client, users):
    dev1 = [u for u in client.get("/api/users").json()
            if u["username"] == "dev1"][0]
    r = client.patch(f"/api/users/{dev1['id']}", json={
        "role": "tester", "password": "reset999"})
    assert r.status_code == 200
    assert r.json()["role"] == "tester"
    client.post("/api/auth/logout")
    login(client, "dev1", "reset999")


def test_cannot_demote_or_deactivate_self(client, admin):
    me = client.get("/api/auth/me").json()
    assert client.patch(f"/api/users/{me['id']}",
                        json={"role": "viewer"}).status_code == 400
    assert client.patch(f"/api/users/{me['id']}",
                        json={"active": False}).status_code == 400


def test_patch_nonexistent_user(client, admin):
    assert client.patch("/api/users/9999",
                        json={"role": "dev"}).status_code == 404
