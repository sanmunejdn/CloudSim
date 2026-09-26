"""依赖 resource/models 的机器人用例；缺失时 skip"""

from __future__ import annotations

import requests
import pytest


def test_robot_tcp_ik_requires_models(api: str, has_robot_models: bool) -> None:
    if not has_robot_models:
        pytest.skip("resource/models missing")
    # 无场景时接口应返回结构化失败，而非 500
    r = requests.post(
        f"{api}/api/robot/tcp-ik",
        json={"deltaMm": [1.0, 0.0, 0.0]},
        timeout=30,
    )
    assert r.status_code in (200, 400)
    body = r.json()
    assert "ok" in body


def test_robot_playback_status(api: str, has_robot_models: bool) -> None:
    if not has_robot_models:
        pytest.skip("resource/models missing")
    r = requests.get(f"{api}/api/robot/playback/status", timeout=15)
    assert r.status_code == 200
    body = r.json()
    assert body.get("ok") is True
    assert "running" in body
