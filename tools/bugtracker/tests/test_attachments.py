"""T6 附件：上传/下载/删除、类型白名单、大小上限。"""
from __future__ import annotations

import io

from conftest import USERS, login, make_bug


def _bug(client) -> int:
    return make_bug(client)["id"]


def test_upload_download_roundtrip(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bid = _bug(client)
    content = b"\x89PNG\r\n\x1a\n fake-image-bytes"
    r = client.post(f"/api/bugs/{bid}/attachments",
                    files={"file": ("shot.png", io.BytesIO(content), "image/png")})
    assert r.status_code == 201
    att = r.json()
    assert att["filename"] == "shot.png"
    assert att["size"] == len(content)

    r = client.get(f"/api/attachments/{att['id']}/download")
    assert r.status_code == 200
    assert r.content == content

    listing = client.get(f"/api/bugs/{bid}/attachments").json()
    assert len(listing) == 1


def test_upload_forbidden_type(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bid = _bug(client)
    r = client.post(f"/api/bugs/{bid}/attachments",
                    files={"file": ("evil.exe", io.BytesIO(b"MZ"), "application/x-msdownload")})
    assert r.status_code == 415


def test_upload_oversize(client, users, monkeypatch):
    login(client, "tester1", USERS["tester1"][0])
    bid = _bug(client)
    from bugtracker.db import db
    monkeypatch.setattr(db.settings, "max_upload_mb", 0)  # 0MB → 任何文件超限
    r = client.post(f"/api/bugs/{bid}/attachments",
                    files={"file": ("big.log", io.BytesIO(b"x" * 100), "text/plain")})
    assert r.status_code == 413
    # 临时文件已清理
    assert list(db.settings.attach_dir.glob("*.part")) == []


def test_viewer_cannot_upload(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bid = _bug(client)
    login(client, "viewer1", USERS["viewer1"][0])
    r = client.post(f"/api/bugs/{bid}/attachments",
                    files={"file": ("a.png", io.BytesIO(b"123"), "image/png")})
    assert r.status_code == 403


def test_delete_permission(client, users):
    login(client, "tester1", USERS["tester1"][0])
    bid = _bug(client)
    att = client.post(f"/api/bugs/{bid}/attachments",
                      files={"file": ("a.log", io.BytesIO(b"log"), "text/plain")}).json()
    login(client, "dev1", USERS["dev1"][0])
    assert client.delete(f"/api/attachments/{att['id']}").status_code == 403
    login(client, "admin", "admin123")
    assert client.delete(f"/api/attachments/{att['id']}").status_code == 200
    assert client.get(
        f"/api/attachments/{att['id']}/download").status_code == 404
