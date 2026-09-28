"""评论：列表 / 发表（@提及触发通知）/ 删除（本人或 admin）。"""
from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy import select
from sqlalchemy.orm import Session

from ..db import get_db
from ..models import Comment, User
from ..schemas import CommentIn
from ..security import current_user
from ..services.notify import notify_bug_change, notify_mentions
from .bugs import get_bug_or_404

router = APIRouter(prefix="/api/bugs/{bug_id}/comments", tags=["comments"])


def _comment_dict(c: Comment) -> dict:
    return {"id": c.id, "bug_id": c.bug_id, "user_id": c.user_id,
            "username": c.user.username if c.user else "",
            "body": c.body,
            "created_at": c.created_at.isoformat() if c.created_at else None}


@router.get("")
def list_comments(bug_id: int, sess: Session = Depends(get_db),
                  user: User = Depends(current_user)):
    get_bug_or_404(sess, bug_id)
    rows = sess.scalars(select(Comment).where(Comment.bug_id == bug_id)
                        .order_by(Comment.id)).all()
    return [_comment_dict(c) for c in rows]


@router.post("", status_code=201)
def add_comment(bug_id: int, body: CommentIn,
                sess: Session = Depends(get_db),
                user: User = Depends(current_user)):
    bug = get_bug_or_404(sess, bug_id)
    if user.role == "viewer":
        raise HTTPException(403, "只读角色不能评论")
    comment = Comment(bug_id=bug_id, user_id=user.id, body=body.body)
    sess.add(comment)
    sess.flush()
    notify_bug_change(sess, bug, user, "发表了评论")
    notify_mentions(sess, bug, user, body.body)
    return _comment_dict(comment)


@router.delete("/{comment_id}")
def delete_comment(bug_id: int, comment_id: int,
                   sess: Session = Depends(get_db),
                   user: User = Depends(current_user)):
    comment = sess.get(Comment, comment_id)
    if comment is None or comment.bug_id != bug_id:
        raise HTTPException(404, "评论不存在")
    if comment.user_id != user.id and user.role != "admin":
        raise HTTPException(403, "只能删除自己的评论")
    sess.delete(comment)
    return {"ok": True}
