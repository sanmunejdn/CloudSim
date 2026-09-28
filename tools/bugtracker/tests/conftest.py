"""pytest 夹具：临时数据目录 + TestClient + 角色用户。"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

PKG_ROOT = Path(__file__).resolve().parent.parent
if str(PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(PKG_ROOT))

ADMIN_PASSWORD = "admin123"
USERS = {  # 用户名: (密码, 角色)
    "dev1": ("dev12345", "dev"),
    "tester1": ("test12345", "tester"),
    "viewer1": ("view12345", "viewer"),
}


@pytest.fixture()
def client(tmp_path, monkeypatch):
    monkeypatch.setenv("BT_DATA_DIR", str(tmp_path / "data"))
    monkeypatch.setenv("BT_INTEGRATION_TOKEN", "test-token")
    monkeypatch.setenv("BT_AGENT_USERNAME", "agent")
    monkeypatch.setenv("BT_ADMIN_PASSWORD", ADMIN_PASSWORD)

    from fastapi.testclient import TestClient
    from bugtracker import db as db_module
    from bugtracker.app import create_app
    from bugtracker.config import load_settings

    db_module.db.__init__()  # 重置进程级单例，指向临时库
    settings = load_settings(env_file=tmp_path / ".env.nonexistent")
    app = create_app(settings)
    with TestClient(app) as c:
        c.tmp_path = tmp_path
        yield c


def login(client, username: str, password: str) -> dict:
    r = client.post("/api/auth/login",
                    json={"username": username, "password": password})
    assert r.status_code == 200, r.text
    return r.json()["user"]


@pytest.fixture()
def admin(client):
    return login(client, "admin", ADMIN_PASSWORD)


@pytest.fixture()
def users(client, admin):
    """admin 建好 dev1/tester1/viewer1 三个账号。"""
    for username, (password, role) in USERS.items():
        r = client.post("/api/users", json={
            "username": username, "password": password, "role": role})
        assert r.status_code == 201, r.text
    return USERS


def make_bug(client, title="示例 bug", project_id=1, **kwargs) -> dict:
    payload = {"project_id": project_id, "title": title,
               "description": "描述", "repro_steps": "步骤",
               "expected": "期望", "actual": "实际"}
    payload.update(kwargs)
    r = client.post("/api/bugs", json=payload)
    assert r.status_code == 201, r.text
    return r.json()
