"""几何建模专用 API 已从网页端移除（暂仅主程序）"""

from __future__ import annotations

import requests


def test_geomodeling_routes_removed(api: str) -> None:
    assert requests.get(f"{api}/api/geomodeling/summary", timeout=10).status_code == 404
    assert requests.post(f"{api}/api/geomodeling/op", json={"op": "primitive"}, timeout=10).status_code == 404
    assert requests.get(f"{api}/api/processflow/graph", timeout=10).status_code == 404
    assert requests.post(f"{api}/api/drawing/export", json={"format": "svg"}, timeout=10).status_code == 404
    assert requests.get(f"{api}/api/labeling/tasks", timeout=10).status_code == 404
