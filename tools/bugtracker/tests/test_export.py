"""T9 导入导出：JSON 全量往返、CSV 导出/导入。"""
from __future__ import annotations

import io

from conftest import USERS, login, make_bug


def test_json_export_import_roundtrip(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bug = make_bug(client, title="导出往返", severity="fatal")
    client.post(f"/api/bugs/{bug['id']}/comments", json={"body": "评论留存"})
    login(client, "admin", "admin123")
    client.post("/api/labels", json={"name": "回归"})

    payload = client.get("/api/export/json").json()
    assert payload["tables"]["bugs"]
    bug_count = len(payload["tables"]["bugs"])
    user_count = len(payload["tables"]["users"])

    # 清空重建
    r = client.post("/api/import/json?confirm=yes", json=payload)
    assert r.status_code == 200, r.text
    assert r.json()["counts"]["bugs"] == bug_count
    assert r.json()["counts"]["users"] == user_count

    # 数据仍在（导入后旧会话已随 users 表重建而失效，重新登录）
    login(client, "admin", "admin123")
    bugs = client.get("/api/bugs").json()
    assert bugs["total"] == bug_count
    titles = {b["title"] for b in bugs["items"]}
    assert "导出往返" in titles


def test_import_json_requires_confirm(client, admin):
    r = client.post("/api/import/json", json={"tables": {}})
    assert r.status_code == 400


def test_import_json_forbidden_for_non_admin(client, users):
    login(client, "dev1", USERS["dev1"][0])
    r = client.post("/api/import/json?confirm=yes", json={"tables": {}})
    assert r.status_code == 403


def test_csv_export(client, users):
    login(client, "tester1", USERS["tester1"][0])
    make_bug(client, title="CSV导出甲")
    make_bug(client, title="CSV导出乙", severity="fatal")
    r = client.get("/api/export/csv")
    assert r.status_code == 200
    text = r.content.decode("utf-8-sig")
    assert "CSV导出甲" in text and "CSV导出乙" in text
    assert "title" in text.splitlines()[0]
    # 带筛选导出
    r = client.get("/api/export/csv?severity=fatal")
    text = r.content.decode("utf-8-sig")
    assert "CSV导出乙" in text and "CSV导出甲" not in text


def test_csv_import(client, admin):
    csv_text = "title,severity,priority,description\n导入甲,fatal,P0,描述甲\n导入乙,,,描述乙\n"
    r = client.post("/api/import/csv?project_id=1",
                    files={"file": ("bugs.csv", io.BytesIO(csv_text.encode()),
                                    "text/csv")})
    assert r.status_code == 200
    assert r.json()["imported"] == 2
    bugs = client.get("/api/bugs?keyword=导入").json()
    assert bugs["total"] == 2
    by_title = {b["title"]: b for b in bugs["items"]}
    assert by_title["导入甲"]["severity"] == "fatal"
    # 非法枚举回退默认
    assert by_title["导入乙"]["severity"] == "major"
    assert by_title["导入乙"]["status"] == "new"


def test_csv_import_bad_project(client, admin):
    csv_text = "title\nx\n"
    r = client.post("/api/import/csv?project_id=999",
                    files={"file": ("b.csv", io.BytesIO(csv_text.encode()),
                                    "text/csv")})
    assert r.status_code == 422
