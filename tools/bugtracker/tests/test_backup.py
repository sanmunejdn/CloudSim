"""在线备份端点。"""
from __future__ import annotations

import sqlite3
from pathlib import Path

from conftest import USERS, login, make_bug


def test_backup_admin(client, admin):
    make_bug(client, title="备份目标")
    r = client.post("/api/backup")
    assert r.status_code == 200, r.text
    body = r.json()
    assert body["ok"] is True
    dest = Path(body["path"])
    assert dest.is_dir()
    db_file = Path(body["db"])
    assert db_file.is_file()

    conn = sqlite3.connect(str(db_file))
    try:
        tables = {row[0] for row in conn.execute(
            "SELECT name FROM sqlite_master WHERE type='table'").fetchall()}
        assert "bugs" in tables
        count = conn.execute("SELECT COUNT(*) FROM bugs").fetchone()[0]
        assert count >= 1
    finally:
        conn.close()


def test_backup_viewer_forbidden(client, users):
    login(client, "viewer1", USERS["viewer1"][0])
    r = client.post("/api/backup")
    assert r.status_code == 403
