"""用户管理：列表（登录可读，供指派下拉）/ 建号 / 改角色 / 停用 / 重置密码（admin）。"""
from __future__ import annotations

from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy import select
from sqlalchemy.orm import Session

from ..db import get_db
from ..models import ROLES, User
from ..schemas import UserCreate, UserOut, UserPatch
from ..security import current_user, hash_password, require

router = APIRouter(prefix="/api/users", tags=["users"])


@router.get("")
def list_users(sess: Session = Depends(get_db),
               user: User = Depends(current_user)) -> list[UserOut]:
    rows = sess.scalars(select(User).order_by(User.id)).all()
    return [UserOut.model_validate(r) for r in rows]


@router.post("", status_code=201)
def create_user(body: UserCreate, sess: Session = Depends(get_db),
                admin: User = Depends(require("admin"))) -> UserOut:
    if body.role not in ROLES:
        raise HTTPException(422, f"role 非法，允许值: {list(ROLES)}")
    if sess.scalar(select(User).where(User.username == body.username)):
        raise HTTPException(409, "用户名已存在")
    user = User(username=body.username,
                password_hash=hash_password(body.password),
                display_name=body.display_name or body.username,
                role=body.role, must_change_password=True)
    sess.add(user)
    sess.flush()
    return UserOut.model_validate(user)


@router.patch("/{user_id}")
def patch_user(user_id: int, body: UserPatch, sess: Session = Depends(get_db),
               admin: User = Depends(require("admin"))) -> UserOut:
    target = sess.get(User, user_id)
    if target is None:
        raise HTTPException(404, "用户不存在")
    if body.role is not None:
        if body.role not in ROLES:
            raise HTTPException(422, f"role 非法，允许值: {list(ROLES)}")
        if target.id == admin.id and body.role != "admin":
            raise HTTPException(400, "不能降级自己的管理员角色")
        target.role = body.role
    if body.active is not None:
        if target.id == admin.id and not body.active:
            raise HTTPException(400, "不能停用自己的账号")
        target.active = body.active
    if body.display_name is not None:
        target.display_name = body.display_name
    if body.password is not None:
        target.password_hash = hash_password(body.password)
        target.must_change_password = True
    sess.flush()
    return UserOut.model_validate(target)
