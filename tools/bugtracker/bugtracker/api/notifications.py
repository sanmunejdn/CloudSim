"""通知中心：列表 / 未读数 / 标记已读。"""
from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException, Query
from sqlalchemy import func, select, update
from sqlalchemy.orm import Session

from ..db import get_db
from ..models import Notification, User
from ..security import current_user

router = APIRouter(prefix="/api/notifications", tags=["notifications"])


def _notif_dict(n: Notification) -> dict:
    return {"id": n.id, "bug_id": n.bug_id, "type": n.type,
            "message": n.message, "read": n.read,
            "created_at": n.created_at.isoformat() if n.created_at else None}


@router.get("")
def list_notifications(unread: bool = False,
                       page: int = Query(default=1, ge=1),
                       size: int = Query(default=50, ge=1, le=200),
                       sess: Session = Depends(get_db),
                       user: User = Depends(current_user)):
    stmt = select(Notification).where(Notification.user_id == user.id)
    if unread:
        stmt = stmt.where(Notification.read.is_(False))
    total = sess.scalar(select(func.count()).select_from(stmt.subquery()))
    rows = sess.scalars(stmt.order_by(Notification.id.desc())
                        .offset((page - 1) * size).limit(size)).all()
    return {"items": [_notif_dict(n) for n in rows], "total": total}


@router.get("/unread-count")
def unread_count(sess: Session = Depends(get_db),
                 user: User = Depends(current_user)):
    count = sess.scalar(
        select(func.count(Notification.id))
        .where(Notification.user_id == user.id, Notification.read.is_(False)))
    return {"count": count}


@router.post("/{notif_id}/read")
def mark_read(notif_id: int, sess: Session = Depends(get_db),
              user: User = Depends(current_user)):
    notif = sess.get(Notification, notif_id)
    if notif is None or notif.user_id != user.id:
        raise HTTPException(404, "通知不存在")
    notif.read = True
    return {"ok": True}


@router.post("/read-all")
def mark_all_read(sess: Session = Depends(get_db),
                  user: User = Depends(current_user)):
    sess.execute(update(Notification)
                 .where(Notification.user_id == user.id,
                        Notification.read.is_(False))
                 .values(read=True))
    return {"ok": True}
