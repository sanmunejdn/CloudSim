"""密码哈希、会话签发/校验、角色权限依赖。"""
from __future__ import annotations

import hashlib
import hmac
import secrets
from datetime import timedelta

from fastapi import Depends, HTTPException, Request
from sqlalchemy.orm import Session as OrmSession

from .db import db, get_db
from .models import Session, User, utcnow

PBKDF2_ITERATIONS = 100_000
COOKIE_NAME = "bt_session"

# 角色等级：require("dev") 表示 dev 及以上（含 admin）
ROLE_RANK = {"viewer": 0, "tester": 1, "dev": 2, "admin": 3}


def hash_password(password: str, salt: str | None = None) -> str:
    salt = salt or secrets.token_hex(16)
    digest = hashlib.pbkdf2_hmac(
        "sha256", password.encode("utf-8"), bytes.fromhex(salt), PBKDF2_ITERATIONS)
    return f"{PBKDF2_ITERATIONS}${salt}${digest.hex()}"


def verify_password(password: str, stored: str) -> bool:
    try:
        iters, salt, expected = stored.split("$")
        digest = hashlib.pbkdf2_hmac(
            "sha256", password.encode("utf-8"), bytes.fromhex(salt), int(iters))
        return hmac.compare_digest(digest.hex(), expected)
    except (ValueError, AttributeError):
        return False


def create_session(sess: OrmSession, user_id: int, ttl_hours: int) -> str:
    token = secrets.token_urlsafe(32)
    sess.add(Session(token=token, user_id=user_id,
                     expires_at=utcnow() + timedelta(hours=ttl_hours)))
    sess.flush()
    return token


def get_session_user(sess: OrmSession, token: str) -> User | None:
    row = sess.get(Session, token)
    if row is None or row.expires_at < utcnow():
        return None
    user = sess.get(User, row.user_id)
    if user is None or not user.active:
        return None
    return user


def current_user(request: Request,
                 sess: OrmSession = Depends(get_db)) -> User:
    token = request.cookies.get(COOKIE_NAME, "")
    user = get_session_user(sess, token) if token else None
    if user is None:
        raise HTTPException(401, "未登录或会话已过期")
    return user


def require(min_role: str):
    """FastAPI 依赖：要求登录用户角色 >= min_role。"""
    def checker(user: User = Depends(current_user)) -> User:
        if ROLE_RANK.get(user.role, -1) < ROLE_RANK[min_role]:
            raise HTTPException(403, "权限不足")
        return user
    return checker


def verify_integration_token(request: Request) -> None:
    """集成接口（git hook / CI）令牌校验。"""
    settings = request.app.state.settings
    token = request.headers.get("X-Integration-Token", "")
    if not settings.integration_token or token != settings.integration_token:
        raise HTTPException(401, "集成令牌无效")
