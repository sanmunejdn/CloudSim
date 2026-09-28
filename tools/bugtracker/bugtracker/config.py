"""配置加载：.env 文件 + BT_* 环境变量（环境变量优先）。"""
from __future__ import annotations

import os
import secrets
from dataclasses import dataclass
from pathlib import Path

# tools/bugtracker/ 目录（包的上一级）
PKG_ROOT = Path(__file__).resolve().parent.parent


def _parse_env_file(path: Path) -> dict[str, str]:
    """解析 KEY=VALUE 行，忽略注释与空行。"""
    values: dict[str, str] = {}
    if not path.is_file():
        return values
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        values[key.strip()] = value.strip().strip('"').strip("'")
    return values


@dataclass
class Settings:
    host: str = "0.0.0.0"
    port: int = 8500
    data_dir: Path = PKG_ROOT / "data"
    db_path: Path = PKG_ROOT / "data" / "bugtracker.db"
    attach_dir: Path = PKG_ROOT / "data" / "attachments"
    log_file: Path = PKG_ROOT / "data" / "server.log"
    secret_key: str = ""
    admin_password: str = ""  # 首启种子 admin 密码；空则随机生成并打印
    session_ttl_hours: int = 12
    max_upload_mb: int = 50
    integration_token: str = ""
    agent_username: str = "agent"
    smtp_host: str = ""
    smtp_port: int = 465
    smtp_user: str = ""
    smtp_password: str = ""
    smtp_from: str = ""

    @property
    def max_upload_bytes(self) -> int:
        return self.max_upload_mb * 1024 * 1024


def _persist_secret(data_dir: Path) -> str:
    """密钥落盘复用，避免重启后全部会话失效。"""
    data_dir.mkdir(parents=True, exist_ok=True)
    secret_file = data_dir / ".secret"
    if secret_file.is_file():
        return secret_file.read_text(encoding="utf-8").strip()
    value = secrets.token_urlsafe(32)
    secret_file.write_text(value, encoding="utf-8")
    return value


def load_settings(env_file: str | os.PathLike[str] | None = None,
                  environ: dict[str, str] | None = None) -> Settings:
    env_path = Path(env_file) if env_file else PKG_ROOT / ".env"
    merged = _parse_env_file(env_path)
    # BT_* 环境变量覆盖 .env
    for key, value in (environ or os.environ).items():
        if key.startswith("BT_"):
            merged[key] = value

    def get(key: str, default: str = "") -> str:
        return merged.get(f"BT_{key}", default)

    def get_int(key: str, default: int) -> int:
        raw = merged.get(f"BT_{key}")
        if raw is None or raw == "":
            return default
        try:
            return int(raw)
        except ValueError:
            return default

    data_dir = Path(get("DATA_DIR") or (PKG_ROOT / "data"))
    settings = Settings(
        host=get("HOST", "0.0.0.0"),
        port=get_int("PORT", 8500),
        data_dir=data_dir,
        db_path=Path(get("DB_PATH") or (data_dir / "bugtracker.db")),
        attach_dir=Path(get("ATTACH_DIR") or (data_dir / "attachments")),
        log_file=Path(get("LOG_FILE") or (data_dir / "server.log")),
        secret_key=get("SECRET_KEY"),
        admin_password=get("ADMIN_PASSWORD"),
        session_ttl_hours=get_int("SESSION_TTL_HOURS", 12),
        max_upload_mb=get_int("MAX_UPLOAD_MB", 50),
        integration_token=get("INTEGRATION_TOKEN"),
        agent_username=get("AGENT_USERNAME", "agent"),
        smtp_host=get("SMTP_HOST"),
        smtp_port=get_int("SMTP_PORT", 465),
        smtp_user=get("SMTP_USER"),
        smtp_password=get("SMTP_PASSWORD"),
        smtp_from=get("SMTP_FROM"),
    )
    if not settings.secret_key:
        settings.secret_key = _persist_secret(settings.data_dir)
    return settings
