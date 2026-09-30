"""LIFECYCLE_CONTRACT 可自动化子集（挂在 run_api_tests → run_tests 默认路径）"""

from __future__ import annotations

import json
import threading
import time

import requests


def test_sse_reconnect_storm_still_alive(api: str) -> None:
    """SSE 反复挂载/断开后进程仍响应（对应 stop/队列清理不拖死）。"""

    def drain_briefly() -> None:
        try:
            with requests.get(f"{api}/api/events", stream=True, timeout=5) as resp:
                assert resp.status_code == 200
                deadline = time.time() + 0.8
                for raw in resp.iter_lines(decode_unicode=True):
                    if time.time() > deadline:
                        break
                    _ = raw
        except (requests.RequestException, TimeoutError):
            pass

    for _ in range(8):
        t = threading.Thread(target=drain_briefly, daemon=True)
        t.start()
        t.join(timeout=3.0)

    r = requests.get(f"{api}/api/help", timeout=10)
    assert r.status_code == 200


def test_path_plan_create_twice_ok(api: str) -> None:
    """连续 createPathPlan：无机器人时允许业务失败，但不得 5xx（清高亮缓存路径可走通）。"""
    body = {"sceneRootBackendId": ""}
    r1 = requests.post(f"{api}/api/robot/path-plan", json=body, timeout=30)
    assert r1.status_code < 500, r1.text[:500]
    r2 = requests.post(f"{api}/api/robot/path-plan", json=body, timeout=30)
    assert r2.status_code < 500, r2.text[:500]
    if r1.status_code == 200 and r2.status_code == 200:
        o1 = r1.json()
        o2 = r2.json()
        assert "pathPlanId" in o1 or "id" in o1
        assert "pathPlanId" in o2 or "id" in o2


def test_trajectory_session_probe(api: str) -> None:
    """轨迹会话探活：生命周期相关 API 面可达。"""
    r = requests.get(f"{api}/api/trajectory/session", timeout=15)
    assert r.status_code < 500
    if r.content:
        try:
            json.loads(r.text)
        except json.JSONDecodeError:
            pass
