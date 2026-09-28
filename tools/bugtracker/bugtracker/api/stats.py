"""统计看板数据：状态/严重级别分布、趋势、模块分布、人员工作量。"""
from __future__ import annotations

from datetime import datetime, timedelta

from fastapi import APIRouter, Depends, Query
from sqlalchemy import func, select
from sqlalchemy.orm import Session

from ..db import get_db
from ..models import Bug, Module, User, utcnow
from ..security import current_user

router = APIRouter(prefix="/api/stats", tags=["stats"])

OPEN_STATUSES = ("new", "confirmed", "in_progress", "reopened", "suspended")


@router.get("/overview")
def overview(sess: Session = Depends(get_db),
             user: User = Depends(current_user)):
    by_status = dict(sess.execute(
        select(Bug.status, func.count()).group_by(Bug.status)).all())
    by_severity = dict(sess.execute(
        select(Bug.severity, func.count()).group_by(Bug.severity)).all())
    open_count = sum(by_status.get(s, 0) for s in OPEN_STATUSES)
    return {
        "by_status": by_status,
        "by_severity": by_severity,
        "open": open_count,
        "closed": by_status.get("closed", 0),
        "total": sum(by_status.values()),
    }


@router.get("/trend")
def trend(days: int = Query(default=30, ge=1, le=365),
          sess: Session = Depends(get_db),
          user: User = Depends(current_user)):
    start = utcnow().date() - timedelta(days=days - 1)
    created = dict(sess.execute(
        select(func.date(Bug.created_at), func.count())
        .where(Bug.created_at >= datetime.combine(start, datetime.min.time()))
        .group_by(func.date(Bug.created_at))).all())
    closed = dict(sess.execute(
        select(func.date(Bug.closed_at), func.count())
        .where(Bug.closed_at.is_not(None),
               Bug.closed_at >= datetime.combine(start, datetime.min.time()))
        .group_by(func.date(Bug.closed_at))).all())
    items = []
    for i in range(days):
        day = (start + timedelta(days=i)).isoformat()
        items.append({"date": day,
                      "created": int(created.get(day, 0)),
                      "closed": int(closed.get(day, 0))})
    return {"days": items}


@router.get("/by-module")
def by_module(sess: Session = Depends(get_db),
              user: User = Depends(current_user)):
    rows = sess.execute(
        select(Bug.module_id, func.count())
        .where(Bug.status.in_(OPEN_STATUSES))
        .group_by(Bug.module_id)).all()
    items = []
    for module_id, count in rows:
        name = "未分类"
        if module_id is not None:
            module = sess.get(Module, module_id)
            name = module.name if module else "未分类"
        items.append({"module_id": module_id, "module_name": name,
                      "open_count": count})
    items.sort(key=lambda x: x["open_count"], reverse=True)
    return {"items": items}


@router.get("/by-user")
def by_user(sess: Session = Depends(get_db),
            user: User = Depends(current_user)):
    assigned = dict(sess.execute(
        select(Bug.assignee_id, func.count())
        .where(Bug.status.in_(OPEN_STATUSES), Bug.assignee_id.is_not(None))
        .group_by(Bug.assignee_id)).all())
    fixed = dict(sess.execute(
        select(Bug.assignee_id, func.count())
        .where(Bug.resolution == "fixed", Bug.assignee_id.is_not(None))
        .group_by(Bug.assignee_id)).all())
    reported = dict(sess.execute(
        select(Bug.reporter_id, func.count()).group_by(Bug.reporter_id)).all())
    user_ids = set(assigned) | set(fixed) | set(reported)
    items = []
    for uid in user_ids:
        u = sess.get(User, uid)
        items.append({
            "user_id": uid,
            "username": u.username if u else str(uid),
            "assigned_open": assigned.get(uid, 0),
            "fixed": fixed.get(uid, 0),
            "reported": reported.get(uid, 0),
        })
    items.sort(key=lambda x: (x["assigned_open"], x["fixed"]), reverse=True)
    return {"items": items}
