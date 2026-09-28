"""数据库引擎、会话管理与种子数据。"""
from __future__ import annotations

import logging
import secrets
import sys
from contextlib import contextmanager

from sqlalchemy import create_engine, event, select
from sqlalchemy.orm import sessionmaker

from .config import Settings
from .models import Base, Project, User

_log = logging.getLogger("bugtracker")


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
            cur.close()

        self.SessionLocal = sessionmaker(
            bind=self.engine, autoflush=False, expire_on_commit=False)
        Base.metadata.create_all(self.engine)
        self._seed()

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
