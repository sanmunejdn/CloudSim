"""schema 基线化与迁移执行。"""
from __future__ import annotations

from bugtracker import migrations as mig
from bugtracker.config import load_settings
from bugtracker.db import db
from bugtracker.migrations import CURRENT_VERSION, get_user_version, set_user_version


def _version() -> int:
    raw = db.engine.raw_connection()
    try:
        return get_user_version(raw)
    finally:
        raw.close()


def test_fresh_db_baselined(client):
    assert _version() == CURRENT_VERSION


def test_legacy_zero_version_baselined(tmp_path, monkeypatch):
    monkeypatch.setenv("BT_DATA_DIR", str(tmp_path / "data"))
    monkeypatch.setenv("BT_ADMIN_PASSWORD", "admin123")
    monkeypatch.setenv("BT_INTEGRATION_TOKEN", "t")
    db.__init__()
    settings = load_settings(env_file=tmp_path / ".env.nonexistent")
    db.init(settings)

    raw = db.engine.raw_connection()
    try:
        set_user_version(raw, 0)
        raw.commit()
        assert get_user_version(raw) == 0
    finally:
        raw.close()

    db._apply_schema_version()
    assert _version() == CURRENT_VERSION


def test_migration_v1_to_v2(tmp_path, monkeypatch):
    monkeypatch.setenv("BT_DATA_DIR", str(tmp_path / "data"))
    monkeypatch.setenv("BT_ADMIN_PASSWORD", "admin123")
    monkeypatch.setenv("BT_INTEGRATION_TOKEN", "t")
    db.__init__()
    settings = load_settings(env_file=tmp_path / ".env.nonexistent")
    db.init(settings)
    assert _version() == 1

    def bump(conn):
        conn.execute(
            "CREATE TABLE IF NOT EXISTS _mig_marker (id INTEGER PRIMARY KEY)")

    monkeypatch.setattr(mig, "CURRENT_VERSION", 2)
    monkeypatch.setattr(mig, "MIGRATIONS", {1: bump})
    # db 模块在 init 时已绑定 CURRENT_VERSION 符号；_apply 走 migrations 包内常量
    monkeypatch.setattr("bugtracker.db.CURRENT_VERSION", 2)

    db._apply_schema_version()
    assert _version() == 2

    raw = db.engine.raw_connection()
    try:
        names = {r[0] for r in raw.execute(
            "SELECT name FROM sqlite_master WHERE type='table'").fetchall()}
        assert "_mig_marker" in names
    finally:
        raw.close()

    # 二次 init 幂等
    db._apply_schema_version()
    assert _version() == 2
