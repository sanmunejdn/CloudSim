"""run_with_retry：locked 重试与最终失败。"""
from __future__ import annotations

import pytest
from sqlalchemy.exc import OperationalError

from bugtracker.db import Database


def test_retry_succeeds_after_locks(monkeypatch):
    db = Database()
    calls = {"n": 0}

    def work():
        calls["n"] += 1
        if calls["n"] < 3:
            raise OperationalError("statement", {},
                                   Exception("database is locked"))
        return "ok"

    delays = []
    monkeypatch.setattr("bugtracker.db.time.sleep",
                        lambda s: delays.append(s))

    assert db.run_with_retry(work) == "ok"
    assert calls["n"] == 3
    assert delays == [0.2, 0.4]


def test_retry_gives_up(monkeypatch):
    db = Database()
    monkeypatch.setattr("bugtracker.db.time.sleep", lambda _s: None)

    def work():
        raise OperationalError("statement", {},
                               Exception("database is locked"))

    with pytest.raises(OperationalError):
        db.run_with_retry(work, attempts=3)


def test_retry_skips_non_locked():
    db = Database()

    def work():
        raise OperationalError("statement", {},
                               Exception("no such table: foo"))

    with pytest.raises(OperationalError, match="no such table"):
        db.run_with_retry(work, attempts=3)
