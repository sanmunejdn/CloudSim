"""在线备份：sqlite3 backup API 保证 WAL 下一致，附件整目录复制。"""
from __future__ import annotations

import logging
import shutil
import sqlite3
from datetime import datetime, timezone
from pathlib import Path

from ..config import Settings
from ..db import db

_log = logging.getLogger("bugtracker.backup")


def create_backup(settings: Settings | None = None) -> dict:
    """备份主库 + attachments 到 data/backups/<时间戳>/，返回路径信息。"""
    settings = settings or db.settings
    if settings is None:
        raise RuntimeError("数据库未初始化")

    stamp = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S")
    dest = settings.data_dir / "backups" / stamp
    dest.mkdir(parents=True, exist_ok=True)

    db_dest = dest / "bugtracker.db"
    src_path = str(settings.db_path)
    # backup API 在线热备，避免直接拷贝主库+wal 撕裂
    src = sqlite3.connect(src_path)
    try:
        dst = sqlite3.connect(str(db_dest))
        try:
            src.backup(dst)
        finally:
            dst.close()
    finally:
        src.close()

    attach_src = settings.attach_dir
    attach_dest = dest / "attachments"
    if attach_src.is_dir():
        shutil.copytree(attach_src, attach_dest, dirs_exist_ok=True)
    else:
        attach_dest.mkdir(exist_ok=True)

    _log.info("备份完成: %s", dest)
    return {
        "ok": True,
        "path": str(dest),
        "db": str(db_dest),
        "attachments": str(attach_dest),
    }
