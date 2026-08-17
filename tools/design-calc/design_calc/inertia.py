from __future__ import annotations

import math
from typing import Any


def inertia_cylinder(payload: dict[str, Any]) -> dict[str, Any]:
	"""空心/实心圆柱绕自身轴惯量 + 平行轴定理。

	J0 = 0.5 m (r_o^2 + r_i^2)；Jx = J0 + m e^2
	"""
	d0_mm = float(payload["d0_mm"])
	d1_mm = float(payload.get("d1_mm", 0.0))
	l_mm = float(payload["L_mm"])
	rho = float(payload.get("rho_kg_m3", 7800.0))
	e_mm = float(payload.get("e_mm", 0.0))
	r0 = d0_mm / 2000.0
	r1 = d1_mm / 2000.0
	length = l_mm / 1000.0
	vol = math.pi * (r0 * r0 - r1 * r1) * length
	if vol <= 0:
		raise ValueError("invalid geometry volume")
	m = rho * vol
	j0 = 0.5 * m * (r0 * r0 + r1 * r1)
	e = e_mm / 1000.0
	jx = j0 + m * e * e
	return {
		"shape": "cylinder",
		"mass_kg": m,
		"J0_kgm2": j0,
		"Jx_kgm2": jx,
		"volume_m3": vol,
	}
