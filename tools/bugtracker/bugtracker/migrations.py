"""轻量 schema 迁移：PRAGMA user_version，零新依赖。

今后改表流程：
1. 改 models.py
2. CURRENT_VERSION += 1
3. 在 MIGRATIONS 加一条 from=旧版本 的迁移函数（DDL/DML）
"""
from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Any

_log = logging.getLogger("bugtracker.migrations")

# 现有 schema 基线；机制引入前的存量库在 init 时直接跳到此版本
CURRENT_VERSION = 1

# 键 = 起始版本，函数执行 N → N+1
MIGRATIONS: dict[int, Callable[[Any], None]] = {
    # 示例：2: lambda conn: conn.execute("ALTER TABLE bugs ADD COLUMN foo TEXT"),
}


def get_user_version(conn) -> int:
    row = conn.execute("PRAGMA user_version").fetchone()
    return int(row[0]) if row else 0


def set_user_version(conn, version: int) -> None:
    # PRAGMA 不能绑参，版本号由本模块控制，无注入风险
    conn.execute(f"PRAGMA user_version = {int(version)}")


def migrate(conn) -> int:
    """按序执行缺失迁移，返回最终版本。每步单独提交后再写 user_version。"""
    version = get_user_version(conn)
    while version < CURRENT_VERSION:
        fn = MIGRATIONS.get(version)
        if fn is None:
            raise RuntimeError(
                f"缺少迁移 {version} → {version + 1}；"
                f"请在 MIGRATIONS 中补充，或检查 CURRENT_VERSION={CURRENT_VERSION}")
        _log.info("执行迁移 %s → %s", version, version + 1)
        fn(conn)
        conn.commit()
        set_user_version(conn, version + 1)
        conn.commit()
        version += 1
    return version
