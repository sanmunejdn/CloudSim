"""数据库引擎、会话管理与种子数据。"""
from __future__ import annotations

import logging
import secrets
import sys
import time
from collections.abc import Callable
from contextlib import contextmanager
from typing import TypeVar

from sqlalchemy import create_engine, event, select
from sqlalchemy.exc import OperationalError
from sqlalchemy.orm import sessionmaker

from .config import Settings
from .migrations import CURRENT_VERSION, get_user_version, migrate, set_user_version
from .models import Base, Project, User

_log = logging.getLogger("bugtracker")

T = TypeVar("T")

# Web/MCP/CLI 多进程写同一库；默认 busy_timeout=0 会立即失败
BUSY_TIMEOUT_MS = 5000
RETRY_ATTEMPTS = 3
RETRY_BACKOFF_SEC = (0.2, 0.4)


def _is_locked(exc: BaseException) -> bool:
    msg = str(exc).lower()
    return "locked" in msg or "busy" in msg


class Database:
    """进程级单例；init 幂等（测试可重复初始化到临时库）。"""

    def __init__(self) -> None:
        self.engine = None
        self.SessionLocal = None
        self.settings: Settings | None = None

    def init(self, settings: Settings) -> None:
        self.settings = settings
        settings.data_dir.mkdir(parents=True, exist_ok=True)
        settings.attach_dir.mkdir(parents=True, exist_ok=True)
        self.engine = create_engine(
            f"sqlite:///{settings.db_path}",
            connect_args={"check_same_thread": False},
        )

        @event.listens_for(self.engine, "connect")
        def _set_pragma(dbapi_conn, _record):
            cur = dbapi_conn.cursor()
            cur.execute("PRAGMA journal_mode=WAL")
            cur.execute("PRAGMA foreign_keys=ON")
            cur.execute(f"PRAGMA busy_timeout={BUSY_TIMEOUT_MS}")
            cur.close()

        self.SessionLocal = sessionmaker(
            bind=self.engine, autoflush=False, expire_on_commit=False)
        Base.metadata.create_all(self.engine)
        self._apply_schema_version()
        self._seed()

    def _apply_schema_version(self) -> None:
        """create_all 只建不迁；user_version=0（新库/存量）直接基线化。"""
        raw = self.engine.raw_connection()
        try:
            version = get_user_version(raw)
            if version == 0:
                set_user_version(raw, CURRENT_VERSION)
                raw.commit()
                _log.info("schema 基线化 user_version=%s", CURRENT_VERSION)
            elif version < CURRENT_VERSION:
                migrate(raw)
            elif version > CURRENT_VERSION:
                raise RuntimeError(
                    f"数据库 schema 版本 {version} 高于代码 {CURRENT_VERSION}，"
                    "请升级 BugTracker")
        finally:
            raw.close()
    @contextmanager
    def session(self):
        sess = self.SessionLocal()
        try:
            yield sess
            sess.commit()
        except Exception:
            sess.rollback()
            raise
        finally:
            sess.close()

    def run_with_retry(self, work: Callable[[], T],
                       attempts: int = RETRY_ATTEMPTS) -> T:
        """整段事务重试：会话已回滚，重试安全。兜底 busy_timeout 盖不住的立即 locked。"""
        last: BaseException | None = None
        for i in range(attempts):
            try:
                return work()
            except OperationalError as exc:
                last = exc
                if not _is_locked(exc) or i >= attempts - 1:
                    raise
                delay = RETRY_BACKOFF_SEC[min(i, len(RETRY_BACKOFF_SEC) - 1)]
                _log.warning("database locked，%.1fs 后重试 (%s/%s)",
                             delay, i + 1, attempts)
                time.sleep(delay)
        raise last  # pragma: no cover

    def _seed(self) -> None:
        # 延迟导入避免与 security 循环依赖
        from .security import hash_password

        with self.session() as sess:
            if not sess.scalar(select(Project).limit(1)):
                sess.add(Project(key="CLOUDSIM", name="CloudSim",
                                 description="默认项目（可在项目页修改）"))

            if not sess.scalar(select(User).where(User.username == "admin")):
                configured = self.settings.admin_password if self.settings else ""
                password = configured or secrets.token_urlsafe(9)
                sess.add(User(username="admin",
                              password_hash=hash_password(password),
                              display_name="管理员", role="admin",
                              must_change_password=True))
                if not configured:
                    # 仅首启打印一次，不落盘；走 stderr 避免污染 MCP stdio 协议流
                    print(f"[BugTracker] 初始管理员账号: admin  随机密码: {password}",
                          file=sys.stderr)
                    print("[BugTracker] 请首次登录后立即修改密码。"
                          "（也可用 BT_ADMIN_PASSWORD 指定初始密码）",
                          file=sys.stderr)

            agent_name = (self.settings.agent_username if self.settings else "agent")
            if not sess.scalar(select(User).where(User.username == agent_name)):
                sess.add(User(username=agent_name,
                              password_hash=hash_password(secrets.token_urlsafe(24)),
                              display_name="AI Agent", role="dev", active=True))


db = Database()


def get_db():
    """FastAPI 依赖：请求级会话，正常结束自动提交。"""
    with db.session() as sess:
        yield sess
