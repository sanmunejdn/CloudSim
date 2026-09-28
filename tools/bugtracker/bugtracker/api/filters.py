"""保存的过滤器（每用户）。"""
from __future__ import annotations

import json

from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy import select
from sqlalchemy.orm import Session

from ..db import get_db
from ..models import SavedFilter, User
from ..schemas import FilterIn
from ..security import current_user

router = APIRouter(prefix="/api/filters", tags=["filters"])


def _filter_dict(f: SavedFilter) -> dict:
    return {"id": f.id, "name": f.name, "query_json": f.query_json,
            "created_at": f.created_at.isoformat() if f.created_at else None}


@router.get("")
def list_filters(sess: Session = Depends(get_db),
                 user: User = Depends(current_user)):
    rows = sess.scalars(select(SavedFilter)
                        .where(SavedFilter.user_id == user.id)
                        .order_by(SavedFilter.id)).all()
    return [_filter_dict(f) for f in rows]


@router.post("", status_code=201)
def create_filter(body: FilterIn, sess: Session = Depends(get_db),
                  user: User = Depends(current_user)):
    try:
        json.loads(body.query_json)
    except json.JSONDecodeError:
        raise HTTPException(422, "query_json 必须是合法 JSON")
    exists = sess.scalar(select(SavedFilter)
                         .where(SavedFilter.user_id == user.id,
                                SavedFilter.name == body.name))
    if exists:
        raise HTTPException(409, "同名过滤器已存在")
    f = SavedFilter(user_id=user.id, name=body.name, query_json=body.query_json)
    sess.add(f)
    sess.flush()
    return _filter_dict(f)


@router.delete("/{filter_id}")
def delete_filter(filter_id: int, sess: Session = Depends(get_db),
                  user: User = Depends(current_user)):
    f = sess.get(SavedFilter, filter_id)
    if f is None or f.user_id != user.id:
        raise HTTPException(404, "过滤器不存在")
    sess.delete(f)
    return {"ok": True}
