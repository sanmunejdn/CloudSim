"""站内通知；SMTP 邮件通道预留（TODO：配置 BT_SMTP_* 后启用）。"""
from __future__ import annotations

import logging
import re
from collections.abc import Iterable

from sqlalchemy import select
from sqlalchemy.orm import Session

from ..models import Bug, BugWatcher, Notification, User

_log = logging.getLogger("bugtracker")

MENTION_RE = re.compile(r"@([A-Za-z0-9_\-一-龥]+)")


def notify(sess: Session, user_ids: Iterable[int], bug_id: int | None,
           ntype: str, message: str, exclude_user_id: int | None = None) -> None:
    targets = {uid for uid in user_ids if uid and uid != exclude_user_id}
    for uid in targets:
        sess.add(Notification(user_id=uid, bug_id=bug_id,
                              type=ntype, message=message))


def bug_stakeholders(sess: Session, bug: Bug) -> set[int]:
    """报告人 + 指派人 + 关注人。"""
    ids = {bug.reporter_id}
    if bug.assignee_id:
        ids.add(bug.assignee_id)
    rows = sess.scalars(
        select(BugWatcher.user_id).where(BugWatcher.bug_id == bug.id)).all()
    ids.update(rows)
    return ids


def notify_bug_change(sess: Session, bug: Bug, actor: User, message: str) -> None:
    notify(sess, bug_stakeholders(sess, bug), bug.id, "bug_change",
           f"#{bug.id} {bug.title}: {message}", exclude_user_id=actor.id)


def notify_mentions(sess: Session, bug: Bug, actor: User, body: str) -> None:
    names = set(MENTION_RE.findall(body or ""))
    if not names:
        return
    users = sess.scalars(
        select(User).where(User.username.in_(names), User.active.is_(True))).all()
    notify(sess, [u.id for u in users], bug.id, "mention",
           f"#{bug.id} {bug.title}: 评论中提到了你", exclude_user_id=actor.id)


def send_email(settings, to_addr: str, subject: str, body: str) -> bool:
    """邮件通道占位。TODO: 在 .env 配置 BT_SMTP_* 后启用真实发送。"""
    if not settings.smtp_host:
        return False
    # TODO: smtplib.SMTP_SSL 发送（密码仅来自 .env，禁止入库/入 git）
    _log.info("SMTP 已配置但邮件发送尚未启用（TODO）: to=%s subject=%s",
              to_addr, subject)
    return False
