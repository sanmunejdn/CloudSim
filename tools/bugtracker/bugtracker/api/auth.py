"""认证：登录 / 登出 / 当前用户 / 修改密码。"""
from __future__ import annotations

from datetime import datetime

from fastapi import APIRouter, Depends, HTTPException, Request, Response
from sqlalchemy import delete, select
from sqlalchemy.orm import Session

from ..db import db, get_db
from ..models import Session as SessionRow
from ..models import User
from ..schemas import LoginIn, PasswordChange, UserOut
from ..security import (COOKIE_NAME, create_session, current_user,
                        hash_password, verify_password)

router = APIRouter(prefix="/api/auth", tags=["auth"])


@router.post("/login")
def login(body: LoginIn, response: Response, sess: Session = Depends(get_db)):
    user = sess.scalar(select(User).where(User.username == body.username))
    if user is None or not verify_password(body.password, user.password_hash):
        raise HTTPException(401, "用户名或密码错误")
    if not user.active:
        raise HTTPException(403, "账号已停用")
    settings = db.settings
    token = create_session(sess, user.id, settings.session_ttl_hours)
    response.set_cookie(COOKIE_NAME, token, httponly=True, samesite="lax",
                        max_age=settings.session_ttl_hours * 3600)
    return {"user": UserOut.model_validate(user)}


@router.post("/logout")
def logout(request: Request, response: Response,
           sess: Session = Depends(get_db),
           user: User = Depends(current_user)):
    token = request.cookies.get(COOKIE_NAME, "")
    if token:
        sess.execute(delete(SessionRow).where(SessionRow.token == token))
    response.delete_cookie(COOKIE_NAME)
    return {"ok": True}


@router.get("/me")
def me(user: User = Depends(current_user)):
    return UserOut.model_validate(user)


@router.post("/password")
def change_password(body: PasswordChange, request: Request,
                    sess: Session = Depends(get_db),
                    user: User = Depends(current_user)):
    if not verify_password(body.old_password, user.password_hash):
        raise HTTPException(400, "原密码错误")
    user.password_hash = hash_password(body.new_password)
    user.must_change_password = False
    # 改密后仅保留当前会话
    current_token = request.cookies.get(COOKIE_NAME, "")
    sess.execute(delete(SessionRow).where(SessionRow.user_id == user.id,
                                          SessionRow.token != current_token))
    return {"ok": True}
