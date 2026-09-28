"""验证连接 pragma：busy_timeout。"""
from __future__ import annotations

from bugtracker.db import BUSY_TIMEOUT_MS, db


def test_busy_timeout_pragma(client):
    with db.engine.connect() as conn:
        raw = conn.connection.dbapi_connection
        row = raw.execute("PRAGMA busy_timeout").fetchone()
    assert row is not None
    assert int(row[0]) == BUSY_TIMEOUT_MS
