"""python -m bugtracker — 启动 Web 服务。"""
from __future__ import annotations

import uvicorn

from .app import create_app
from .config import load_settings


def main() -> None:
    settings = load_settings()
    app = create_app(settings)
    uvicorn.run(app, host=settings.host, port=settings.port, log_level="info")


if __name__ == "__main__":
    main()
