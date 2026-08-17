from __future__ import annotations

from typing import Any


def worm_geometry(payload: dict[str, Any]) -> dict[str, Any]:
	"""圆柱蜗杆传动基本几何（阿基米德蜗杆常用式）。

	d1=q*m，d2=m*z2，a=0.5*(d1+d2)，γ=arctan(z1/q)
	"""
	import math

	m = float(payload["m_mm"])
	z1 = int(payload.get("Z1", 1))
	z2 = int(payload["Z2"])
	q = float(payload.get("q", 10.0))
	ha_star = float(payload.get("ha_star", 1.0))
	c_star = float(payload.get("c_star", 0.2))
	b = float(payload.get("b_mm", 0.0))

	d1 = q * m
	d2 = m * z2
	a = 0.5 * (d1 + d2)
	gamma = math.degrees(math.atan2(z1, q))
	da1 = d1 + 2.0 * ha_star * m
	df1 = d1 - 2.0 * (ha_star + c_star) * m
	da2 = d2 + 2.0 * ha_star * m
	df2 = d2 - 2.0 * (ha_star + c_star) * m
	if b <= 0:
		b = 0.75 * d1
	return {
		"m_mm": m,
		"q": q,
		"Z1": z1,
		"Z2": z2,
		"i": z2 / float(z1),
		"gamma_deg": gamma,
		"A_mm": a,
		"worm": {"d_mm": d1, "da_mm": da1, "df_mm": df1, "length_suggest_mm": b + 2.0 * m},
		"wheel": {"d_mm": d2, "da_mm": da2, "df_mm": df2, "b_mm": b},
	}
