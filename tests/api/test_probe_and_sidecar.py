"""探活 / SSE / sidecar / collision-settings"""

from __future__ import annotations

import json
import threading
import time

import requests


def test_help_probe(api: str) -> None:
    r = requests.get(f"{api}/api/help", timeout=10)
    assert r.status_code == 200
    body = r.json()
    assert "entries" in body or "helpRoot" in body


def test_sse_workspace_mode_event(api: str) -> None:
    got = {"event": None}

    def reader() -> None:
        with requests.get(f"{api}/api/events", stream=True, timeout=30) as resp:
            assert resp.status_code == 200
            for raw in resp.iter_lines(decode_unicode=True):
                if not raw or not raw.startswith("data:"):
                    continue
                payload = raw[5:].strip()
                try:
                    obj = json.loads(payload)
                except json.JSONDecodeError:
                    continue
                if obj.get("type") == "WorkspaceModeChanged":
                    got["event"] = obj
                    return

    t = threading.Thread(target=reader, daemon=True)
    t.start()
    time.sleep(0.5)
    put = requests.put(
        f"{api}/api/sidecar/workspaceMode",
        json={"mode": "geomodeling"},
        timeout=10,
    )
    assert put.status_code == 200
    assert put.json().get("ok") is True
    t.join(timeout=15)
    assert got["event"] is not None
    assert got["event"].get("mode") == "geomodeling"


def test_sidecar_roundtrip(api: str) -> None:
    payload = {"note": "api-test", "v": 1}
    put = requests.put(f"{api}/api/sidecar/geometricModeling", json=payload, timeout=10)
    assert put.status_code == 200
    assert put.json().get("ok") is True
    get = requests.get(f"{api}/api/sidecar/geometricModeling", timeout=10)
    assert get.status_code == 200
    assert get.json().get("note") == "api-test"


def test_collision_settings_roundtrip(api: str) -> None:
    get0 = requests.get(f"{api}/api/robot/collision-settings", timeout=10)
    assert get0.status_code == 200
    body = get0.json()
    assert body.get("ok") is True
    margin = float(body.get("securityMarginMm", 0.0))
    new_margin = margin + 1.0 if margin < 50 else margin - 1.0
    put = requests.put(
        f"{api}/api/robot/collision-settings",
        json={**{k: v for k, v in body.items() if k != "ok"}, "securityMarginMm": new_margin},
        timeout=10,
    )
    assert put.status_code == 200
    assert put.json().get("ok") is True
    get1 = requests.get(f"{api}/api/robot/collision-settings", timeout=10)
    assert abs(float(get1.json().get("securityMarginMm", -1)) - new_margin) < 1e-6
