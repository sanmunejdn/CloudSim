"""FastAPI 应用工厂。"""
from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path

from fastapi import FastAPI, Request
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from . import __version__
from .config import Settings, load_settings
from .db import db

STATIC_DIR = Path(__file__).resolve().parent / "static"


def _setup_logging(settings: Settings) -> None:
    settings.data_dir.mkdir(parents=True, exist_ok=True)
    root = logging.getLogger("bugtracker")
    if root.handlers:
        return
    root.setLevel(logging.INFO)
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(name)s %(message)s")
    # 无限增长会占满磁盘；5MB × 3 代足够排查
    file_handler = RotatingFileHandler(
        settings.log_file, maxBytes=5 * 1024 * 1024, backupCount=3,
        encoding="utf-8")
    file_handler.setFormatter(formatter)
    root.addHandler(file_handler)
    root.addHandler(logging.StreamHandler())


def create_app(settings: Settings | None = None) -> FastAPI:
    settings = settings or load_settings()
    _setup_logging(settings)
    if db.engine is None:
        db.init(settings)

    app = FastAPI(title="BugTracker", version=__version__,
                  docs_url="/api/docs", openapi_url="/api/openapi.json")
    app.state.settings = settings

    from .api import register_routers
    register_routers(app)

    app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

    @app.get("/", include_in_schema=False)
    def index():
        return FileResponse(STATIC_DIR / "index.html")

    @app.get("/api/health", tags=["meta"])
    def health():
        return {"ok": True, "version": __version__}

    @app.exception_handler(Exception)
    async def unhandled_exc(request: Request, exc: Exception):
        logging.getLogger("bugtracker").exception(
            "未处理异常: %s %s", request.method, request.url.path)
        return JSONResponse(status_code=500, content={"detail": "服务器内部错误"})

    return app
