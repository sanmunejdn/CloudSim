"""API 路由注册。新增路由模块时追加到 ROUTERS 列表。"""
from __future__ import annotations

from fastapi import FastAPI


def register_routers(app: FastAPI) -> None:
    from . import (admin, attachments, auth, bugs, comments, filters,
                   integrations, notifications, projects, stats, users)
    for module in (auth, users, projects, bugs, comments, attachments, stats,
                   notifications, filters, integrations, admin):
        app.include_router(module.router)
