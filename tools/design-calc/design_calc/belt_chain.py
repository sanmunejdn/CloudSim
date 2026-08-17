from __future__ import annotations

import math
from typing import Any


def belt_v_length(payload: dict[str, Any]) -> dict[str, Any]:
	"""三角带基准长度近似：L ≈ 2a + π(D1+D2)/2 + (D2-D1)²/(4a)。"""
	a = float(payload["a_mm"])
	d1 = float(payload["D1_mm"])
	d2 = float(payload["D2_mm"])
	if a <= 0:
		raise ValueError("a_mm must be > 0")
	length = 2.0 * a + math.pi * (d1 + d2) / 2.0 + (d2 - d1) ** 2 / (4.0 * a)
	return {"L_mm": length, "a_mm": a, "D1_mm": d1, "D2_mm": d2}


def chain_sprocket_pitch_diameter(payload: dict[str, Any]) -> dict[str, Any]:
	"""链轮分度圆直径 d = p / sin(π/Z)。"""
	p = float(payload["p_mm"])
	z = int(payload["Z"])
	if z < 3 or p <= 0:
		raise ValueError("invalid p/Z")
	d = p / math.sin(math.pi / z)
	da = p * (0.5 + 1.0 / math.tan(math.pi / z))
	return {"p_mm": p, "Z": z, "d_mm": d, "da_approx_mm": da}
