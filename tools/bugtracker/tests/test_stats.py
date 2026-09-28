"""T8 统计 API：构造数据集校验精确数值。"""
from __future__ import annotations

from conftest import USERS, login, make_bug


def _uid(client, username) -> int:
    return [u for u in client.get("/api/users").json()
            if u["username"] == username][0]["id"]


def _dataset(client):
    """4 单：A fatal new（几何模块，指派 dev1）、B major fixed（dev1）、
    C minor closed、D minor new（未分类）。"""
    login(client, "admin", "admin123")
    mid = client.post("/api/projects/1/modules",
                      json={"name": "几何"}).json()["id"]
    login(client, "tester1", USERS["tester1"][0])
    make_bug(client, title="A", severity="fatal", module_id=mid,
             assignee_id=_uid(client, "dev1"))
    b2 = make_bug(client, title="B", severity="major",
                  assignee_id=_uid(client, "dev1"))
    b3 = make_bug(client, title="C", severity="minor")
    make_bug(client, title="D", severity="minor")
    login(client, "dev1", USERS["dev1"][0])
    for bid in (b2["id"], b3["id"]):
        client.post(f"/api/bugs/{bid}/status", json={"to": "confirmed"})
        client.post(f"/api/bugs/{bid}/status", json={"to": "in_progress"})
        client.post(f"/api/bugs/{bid}/status", json={"to": "fixed"})
    login(client, "tester1", USERS["tester1"][0])
    client.post(f"/api/bugs/{b3['id']}/status", json={"to": "verified"})
    client.post(f"/api/bugs/{b3['id']}/status", json={"to": "closed"})
    return mid


def test_overview(client, users):
    _dataset(client)
    r = client.get("/api/stats/overview").json()
    assert r["total"] == 4
    assert r["by_status"]["new"] == 2
    assert r["by_status"]["fixed"] == 1
    assert r["by_status"]["closed"] == 1
    assert r["by_severity"]["fatal"] == 1
    assert r["open"] == 2      # A、D（fixed/verified/closed 不算 open）
    assert r["closed"] == 1


def test_trend(client, users):
    _dataset(client)
    r = client.get("/api/stats/trend?days=7").json()
    assert len(r["days"]) == 7
    today = r["days"][-1]
    assert today["created"] == 4
    assert today["closed"] == 1


def test_by_module(client, users):
    _dataset(client)
    items = client.get("/api/stats/by-module").json()["items"]
    by_name = {i["module_name"]: i["open_count"] for i in items}
    # 仅统计 open：A（几何）、D（未分类）；B fixed、C closed 不计
    assert by_name["几何"] == 1
    assert by_name["未分类"] == 1


def test_by_user(client, users):
    _dataset(client)
    items = client.get("/api/stats/by-user").json()["items"]
    dev1 = [i for i in items if i["username"] == "dev1"][0]
    tester1 = [i for i in items if i["username"] == "tester1"][0]
    assert dev1["assigned_open"] == 1   # A
    assert dev1["fixed"] == 1           # B（C 未指派不计）
    assert tester1["reported"] == 4
