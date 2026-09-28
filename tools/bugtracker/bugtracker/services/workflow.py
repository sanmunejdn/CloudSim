"""Bug 状态机与字段审计 —— 全系统唯一权威（API / MCP / CLI 共用）。"""
from __future__ import annotations

from fastapi import HTTPException
from sqlalchemy.orm import Session

from ..models import (ENVIRONMENTS, PRIORITIES, RESOLUTIONS, SEVERITIES,
                      AuditLog, Bug, Comment, User, utcnow)
from ..security import ROLE_RANK

# 状态机：当前状态 -> {目标状态: 所需最低角色}
TRANSITIONS: dict[str, dict[str, str]] = {
    "new":         {"confirmed": "dev", "rejected": "dev", "duplicate": "dev"},
    "confirmed":   {"in_progress": "dev", "suspended": "dev",
                    "rejected": "dev", "duplicate": "dev"},
    "in_progress": {"fixed": "dev", "suspended": "dev", "confirmed": "dev"},
    "suspended":   {"in_progress": "dev", "confirmed": "dev"},
    "fixed":       {"verified": "tester", "reopened": "tester"},
    "verified":    {"closed": "tester", "reopened": "tester"},
    "reopened":    {"in_progress": "dev", "confirmed": "dev"},
    "rejected":    {"reopened": "tester"},
    "duplicate":   {"reopened": "tester"},
    "closed":      {},
}

# 允许审计的字段（PATCH 与 MCP/CLI 更新共用）
AUDIT_FIELDS = (
    "title", "module_id", "milestone_id", "description", "repro_steps",
    "expected", "actual", "severity", "priority", "environment",
    "version_found", "version_fixed", "assignee_id",
)

_REJECT_RESOLUTIONS = ("wontfix", "bydesign", "cannot_reproduce")


def _has_role(user: User, min_role: str) -> bool:
    return ROLE_RANK.get(user.role, -1) >= ROLE_RANK[min_role]


def validate_enums(severity: str | None = None, priority: str | None = None,
                   environment: str | None = None) -> None:
    if severity is not None and severity not in SEVERITIES:
        raise HTTPException(422, f"severity 非法，允许值: {list(SEVERITIES)}")
    if priority is not None and priority not in PRIORITIES:
        raise HTTPException(422, f"priority 非法，允许值: {list(PRIORITIES)}")
    if environment is not None and environment not in ENVIRONMENTS:
        raise HTTPException(422, f"environment 非法，允许值: {list(ENVIRONMENTS)}")


def audit(sess: Session, bug_id: int, user_id: int, field: str,
          old, new) -> None:
    sess.add(AuditLog(bug_id=bug_id, user_id=user_id, field=field,
                      old_value="" if old is None else str(old),
                      new_value="" if new is None else str(new)))


def transition(sess: Session, bug: Bug, to: str, user: User,
               resolution: str | None = None, comment: str = "") -> Bug:
    """状态流转：校验状态机与权限 → 写审计 → 触发通知。"""
    from .notify import notify_bug_change

    allowed = TRANSITIONS.get(bug.status, {})
    if to not in allowed:
        raise HTTPException(409, {
            "detail": f"不允许从 {bug.status} 流转到 {to}",
            "allowed": sorted(allowed),
        })

    min_role = allowed[to]
    is_reporter_reopen = (to == "reopened" and bug.reporter_id == user.id)
    if not _has_role(user, min_role) and not is_reporter_reopen:
        raise HTTPException(403, f"该流转需要 {min_role} 及以上角色")

    if to == "rejected" and resolution not in _REJECT_RESOLUTIONS:
        raise HTTPException(
            422, f"拒绝时必须给出 resolution: {list(_REJECT_RESOLUTIONS)}")
    if resolution is not None and resolution not in RESOLUTIONS:
        raise HTTPException(422, f"resolution 非法，允许值: {list(RESOLUTIONS)}")

    old_status = bug.status
    bug.status = to
    if to == "fixed":
        bug.resolution = "fixed"
    elif to == "duplicate":
        bug.resolution = "duplicate"
    elif to == "rejected":
        bug.resolution = resolution
    elif to in ("reopened", "in_progress", "confirmed", "suspended"):
        bug.resolution = None

    if to == "closed":
        bug.closed_at = utcnow()
    elif to == "reopened":
        bug.closed_at = None

    audit(sess, bug.id, user.id, "status", old_status, to)
    if comment:
        sess.add(Comment(bug_id=bug.id, user_id=user.id, body=comment))
    sess.flush()

    from .notify import notify_mentions
    notify_bug_change(sess, bug, user, f"状态 {old_status} → {to}")
    if comment:
        notify_mentions(sess, bug, user, comment)
    return bug


def update_fields(sess: Session, bug: Bug, changes: dict, user: User) -> list[str]:
    """字段修改：逐字段比对写审计，返回实际变更的字段名列表。"""
    from .notify import notify_bug_change

    changed: list[str] = []
    for field, value in changes.items():
        if field not in AUDIT_FIELDS:
            raise HTTPException(422, f"字段 {field} 不允许修改")
        old = getattr(bug, field)
        if old != value:
            setattr(bug, field, value)
            audit(sess, bug.id, user.id, field, old, value)
            changed.append(field)
    if changed:
        sess.flush()
        notify_bug_change(sess, bug, user, "修改字段: " + ", ".join(changed))
    return changed
