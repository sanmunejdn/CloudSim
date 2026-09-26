"""几何建模核心 op 回归"""

from __future__ import annotations

from pathlib import Path

import requests


def _op(api: str, body: dict) -> dict:
    r = requests.post(f"{api}/api/geomodeling/op", json=body, timeout=60)
    assert r.status_code == 200, r.text
    data = r.json()
    assert data.get("ok") is True, data
    return data


def _summary(api: str) -> dict:
    r = requests.get(f"{api}/api/geomodeling/summary", timeout=30)
    assert r.status_code == 200
    data = r.json()
    assert data.get("ok") is True
    return data


def test_primitive_box_creates_body(api: str) -> None:
    before = _summary(api)["count"]
    out = _op(api, {"op": "primitive", "kind": "box", "lengthMm": 80, "widthMm": 60, "heightMm": 40})
    assert out.get("backendId")
    assert out.get("hasGeometry") is True
    after = _summary(api)
    assert after["count"] == before + 1
    bodies = after["bodies"]
    assert any(b.get("backendId") == out["backendId"] for b in bodies)


def test_extrude_and_mesh_nonempty(api: str) -> None:
    out = _op(
        api,
        {
            "op": "extrude",
            "mode": "pad",
            "profile": "rectangle",
            "plane": "XY",
            "lengthMm": 50,
            "widthMm": 40,
            "heightMm": 30,
        },
    )
    bid = out["backendId"]
    mesh = requests.get(f"{api}/api/mesh/{bid}", timeout=60)
    assert mesh.status_code == 200
    assert mesh.headers.get("Content-Type", "").startswith("application/octet-stream")
    assert len(mesh.content) >= 9 * 4  # 至少一三角


def test_patch_length_and_undo_redo(api: str) -> None:
    out = _op(api, {"op": "primitive", "kind": "box", "lengthMm": 100, "widthMm": 100, "heightMm": 50})
    bid = out["backendId"]
    summary = _summary(api)
    body = next(b for b in summary["bodies"] if b["backendId"] == bid)
    pad = next(f for f in body["features"] if f.get("kind") == "Pad")
    fid = pad["id"]
    patched = _op(api, {"op": "patch", "backendId": bid, "featureId": fid, "lengthMm": 75.0})
    assert patched.get("ok") is True
    summary2 = _summary(api)
    body2 = next(b for b in summary2["bodies"] if b["backendId"] == bid)
    pad2 = next(f for f in body2["features"] if f["id"] == fid)
    assert abs(float(pad2["lengthMm"]) - 75.0) < 1e-6
    undo_before = summary2.get("undoCount", 0)
    und = _op(api, {"op": "undo"})
    assert und.get("undoCount", 0) <= undo_before
    _op(api, {"op": "redo"})


def test_export_import_history_roundtrip(api: str, tmp_path: Path) -> None:
    out = _op(api, {"op": "primitive", "kind": "cylinder", "radiusMm": 25, "heightMm": 60})
    bid = out["backendId"]
    path = tmp_path / "history.json"
    _op(api, {"op": "exportHistory", "backendId": bid, "path": str(path)})
    assert path.is_file() and path.stat().st_size > 10
    imported = _op(api, {"op": "importHistory", "path": str(path), "createNew": True})
    assert imported.get("backendId")
    assert imported.get("backendId") != bid
