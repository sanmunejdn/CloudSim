"""ORM 数据模型（SQLAlchemy 2.0）。状态/枚举取值见 services/workflow.py 与本模块常量。"""
from __future__ import annotations

from datetime import datetime, timezone

from sqlalchemy import ForeignKey, Integer, String, Text, Boolean, DateTime, UniqueConstraint
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column, relationship


def utcnow() -> datetime:
    return datetime.now(timezone.utc).replace(tzinfo=None)


# ---- 枚举取值（与前端、schemas、workflow 保持一致） ----

ROLES = ("admin", "dev", "tester", "viewer")
SEVERITIES = ("fatal", "major", "minor", "suggestion")
PRIORITIES = ("P0", "P1", "P2", "P3")
BUG_STATUSES = (
    "new", "confirmed", "in_progress", "fixed", "verified",
    "closed", "reopened", "rejected", "duplicate", "suspended",
)
RESOLUTIONS = ("fixed", "wontfix", "duplicate", "bydesign", "cannot_reproduce")
ENVIRONMENTS = ("debug", "release")
MILESTONE_STATUSES = ("open", "closed")


class Base(DeclarativeBase):
    pass


class User(Base):
    __tablename__ = "users"

    id: Mapped[int] = mapped_column(primary_key=True)
    username: Mapped[str] = mapped_column(String(64), unique=True, index=True)
    password_hash: Mapped[str] = mapped_column(String(256))
    display_name: Mapped[str] = mapped_column(String(64), default="")
    role: Mapped[str] = mapped_column(String(16), default="viewer")
    active: Mapped[bool] = mapped_column(Boolean, default=True)
    must_change_password: Mapped[bool] = mapped_column(Boolean, default=False)
    created_at: Mapped[datetime] = mapped_column(DateTime, default=utcnow)


class Session(Base):
    __tablename__ = "sessions"

    token: Mapped[str] = mapped_column(String(128), primary_key=True)
    user_id: Mapped[int] = mapped_column(ForeignKey("users.id", ondelete="CASCADE"))
    created_at: Mapped[datetime] = mapped_column(DateTime, default=utcnow)
    expires_at: Mapped[datetime] = mapped_column(DateTime)


class Project(Base):
    __tablename__ = "projects"

    id: Mapped[int] = mapped_column(primary_key=True)
    key: Mapped[str] = mapped_column(String(32), unique=True, index=True)
    name: Mapped[str] = mapped_column(String(128))
    description: Mapped[str] = mapped_column(Text, default="")
    created_at: Mapped[datetime] = mapped_column(DateTime, default=utcnow)

    modules: Mapped[list["Module"]] = relationship(back_populates="project",
                                                   cascade="all, delete-orphan")
    milestones: Mapped[list["Milestone"]] = relationship(back_populates="project",
                                                         cascade="all, delete-orphan")


class Module(Base):
    __tablename__ = "modules"

    id: Mapped[int] = mapped_column(primary_key=True)
    project_id: Mapped[int] = mapped_column(ForeignKey("projects.id", ondelete="CASCADE"))
    parent_id: Mapped[int | None] = mapped_column(ForeignKey("modules.id", ondelete="CASCADE"),
                                                  nullable=True)
    name: Mapped[str] = mapped_column(String(128))
    sort: Mapped[int] = mapped_column(Integer, default=0)

    project: Mapped[Project] = relationship(back_populates="modules")
    children: Mapped[list["Module"]] = relationship(
        back_populates="parent", cascade="all, delete-orphan",
    )
    parent: Mapped["Module | None"] = relationship(
        back_populates="children", remote_side="Module.id",
    )


class Milestone(Base):
    __tablename__ = "milestones"

    id: Mapped[int] = mapped_column(primary_key=True)
    project_id: Mapped[int] = mapped_column(ForeignKey("projects.id", ondelete="CASCADE"))
    name: Mapped[str] = mapped_column(String(128))
    due_date: Mapped[str] = mapped_column(String(10), default="")  # YYYY-MM-DD
    status: Mapped[str] = mapped_column(String(16), default="open")

    project: Mapped[Project] = relationship(back_populates="milestones")


class Label(Base):
    __tablename__ = "labels"

    id: Mapped[int] = mapped_column(primary_key=True)
    name: Mapped[str] = mapped_column(String(64), unique=True)
    color: Mapped[str] = mapped_column(String(16), default="#888888")


class Bug(Base):
    __tablename__ = "bugs"

    id: Mapped[int] = mapped_column(primary_key=True)
    project_id: Mapped[int] = mapped_column(ForeignKey("projects.id"))
    module_id: Mapped[int | None] = mapped_column(ForeignKey("modules.id"), nullable=True)
    milestone_id: Mapped[int | None] = mapped_column(ForeignKey("milestones.id"), nullable=True)
    title: Mapped[str] = mapped_column(String(256))
    description: Mapped[str] = mapped_column(Text, default="")
    repro_steps: Mapped[str] = mapped_column(Text, default="")
    expected: Mapped[str] = mapped_column(Text, default="")
    actual: Mapped[str] = mapped_column(Text, default="")
    severity: Mapped[str] = mapped_column(String(16), default="major")
    priority: Mapped[str] = mapped_column(String(4), default="P2")
    status: Mapped[str] = mapped_column(String(16), default="new", index=True)
    resolution: Mapped[str | None] = mapped_column(String(32), nullable=True)
    environment: Mapped[str] = mapped_column(String(16), default="debug")
    version_found: Mapped[str] = mapped_column(String(64), default="")
    version_fixed: Mapped[str] = mapped_column(String(64), default="")
    reporter_id: Mapped[int] = mapped_column(ForeignKey("users.id"))
    assignee_id: Mapped[int | None] = mapped_column(ForeignKey("users.id"), nullable=True)
    created_at: Mapped[datetime] = mapped_column(DateTime, default=utcnow)
    updated_at: Mapped[datetime] = mapped_column(DateTime, default=utcnow, onupdate=utcnow)
    closed_at: Mapped[datetime | None] = mapped_column(DateTime, nullable=True)

    reporter: Mapped[User] = relationship(foreign_keys=[reporter_id])
    assignee: Mapped[User | None] = relationship(foreign_keys=[assignee_id])
    labels: Mapped[list[Label]] = relationship(secondary="bug_labels")


class BugLabel(Base):
    __tablename__ = "bug_labels"
    __table_args__ = (UniqueConstraint("bug_id", "label_id"),)

    bug_id: Mapped[int] = mapped_column(ForeignKey("bugs.id", ondelete="CASCADE"),
                                        primary_key=True)
    label_id: Mapped[int] = mapped_column(ForeignKey("labels.id", ondelete="CASCADE"),
                                          primary_key=True)


class Comment(Base):
    __tablename__ = "comments"

    id: Mapped[int] = mapped_column(primary_key=True)
    bug_id: Mapped[int] = mapped_column(ForeignKey("bugs.id", ondelete="CASCADE"), index=True)
    user_id: Mapped[int] = mapped_column(ForeignKey("users.id"))
    body: Mapped[str] = mapped_column(Text)
    created_at: Mapped[datetime] = mapped_column(DateTime, default=utcnow)

    user: Mapped[User] = relationship()


class Attachment(Base):
    __tablename__ = "attachments"

    id: Mapped[int] = mapped_column(primary_key=True)
    bug_id: Mapped[int] = mapped_column(ForeignKey("bugs.id", ondelete="CASCADE"), index=True)
    filename: Mapped[str] = mapped_column(String(256))
    stored_name: Mapped[str] = mapped_column(String(256), unique=True)
    size: Mapped[int] = mapped_column(Integer)
    mime: Mapped[str] = mapped_column(String(128), default="application/octet-stream")
    uploaded_by: Mapped[int] = mapped_column(ForeignKey("users.id"))
    created_at: Mapped[datetime] = mapped_column(DateTime, default=utcnow)


class AuditLog(Base):
    __tablename__ = "audit_logs"

    id: Mapped[int] = mapped_column(primary_key=True)
    bug_id: Mapped[int] = mapped_column(ForeignKey("bugs.id", ondelete="CASCADE"), index=True)
    user_id: Mapped[int] = mapped_column(ForeignKey("users.id"))
    field: Mapped[str] = mapped_column(String(64))
    old_value: Mapped[str] = mapped_column(Text, default="")
    new_value: Mapped[str] = mapped_column(Text, default="")
    created_at: Mapped[datetime] = mapped_column(DateTime, default=utcnow)

    user: Mapped[User] = relationship()


class BugWatcher(Base):
    __tablename__ = "bug_watchers"
    __table_args__ = (UniqueConstraint("bug_id", "user_id"),)

    bug_id: Mapped[int] = mapped_column(ForeignKey("bugs.id", ondelete="CASCADE"),
                                        primary_key=True)
    user_id: Mapped[int] = mapped_column(ForeignKey("users.id", ondelete="CASCADE"),
                                         primary_key=True)


class CommitLink(Base):
    __tablename__ = "commit_links"

    id: Mapped[int] = mapped_column(primary_key=True)
    bug_id: Mapped[int] = mapped_column(ForeignKey("bugs.id", ondelete="CASCADE"), index=True)
    repo: Mapped[str] = mapped_column(String(256), default="")
    commit_hash: Mapped[str] = mapped_column(String(64))
    message: Mapped[str] = mapped_column(Text, default="")
    created_at: Mapped[datetime] = mapped_column(DateTime, default=utcnow)


class TestLink(Base):
    __tablename__ = "test_links"

    id: Mapped[int] = mapped_column(primary_key=True)
    bug_id: Mapped[int] = mapped_column(ForeignKey("bugs.id", ondelete="CASCADE"), index=True)
    test_path: Mapped[str] = mapped_column(String(512))
    note: Mapped[str] = mapped_column(Text, default="")
    created_at: Mapped[datetime] = mapped_column(DateTime, default=utcnow)


class Notification(Base):
    __tablename__ = "notifications"

    id: Mapped[int] = mapped_column(primary_key=True)
    user_id: Mapped[int] = mapped_column(ForeignKey("users.id", ondelete="CASCADE"), index=True)
    bug_id: Mapped[int | None] = mapped_column(ForeignKey("bugs.id", ondelete="CASCADE"),
                                               nullable=True)
    type: Mapped[str] = mapped_column(String(32))
    message: Mapped[str] = mapped_column(Text, default="")
    read: Mapped[bool] = mapped_column(Boolean, default=False, index=True)
    created_at: Mapped[datetime] = mapped_column(DateTime, default=utcnow)


class SavedFilter(Base):
    __tablename__ = "saved_filters"
    __table_args__ = (UniqueConstraint("user_id", "name"),)

    id: Mapped[int] = mapped_column(primary_key=True)
    user_id: Mapped[int] = mapped_column(ForeignKey("users.id", ondelete="CASCADE"))
    name: Mapped[str] = mapped_column(String(64))
    query_json: Mapped[str] = mapped_column(Text, default="{}")
    created_at: Mapped[datetime] = mapped_column(DateTime, default=utcnow)


class Setting(Base):
    __tablename__ = "settings"

    key: Mapped[str] = mapped_column(String(64), primary_key=True)
    value: Mapped[str] = mapped_column(Text, default="")
