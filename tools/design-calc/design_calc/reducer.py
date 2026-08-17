from __future__ import annotations

from typing import Any


def reducer_rated_power(payload: dict[str, Any]) -> dict[str, Any]:
	"""减速机公称功率验算：P2m=P×KA×KS，i=N/n。"""
	ka = float(payload.get("KA", 1.5))
	ks = float(payload.get("KS", 1.5))
	p = float(payload["P_kW"])
	n_in = float(payload["N_rpm"])
	n_out = float(payload["n_rpm"])
	if n_out <= 0:
		raise ValueError("n_rpm must be > 0")
	p2m = p * ka * ks
	ratio = n_in / n_out
	p1_catalog = payload.get("P1_catalog_kW")
	ok = None
	if p1_catalog is not None:
		ok = float(p1_catalog) + 1e-9 >= p2m
	return {
		"P2m_kW": p2m,
		"i": ratio,
		"P1_catalog_kW": p1_catalog,
		"pass": ok,
		"note": "P2m < P1（样本）" if ok else ("需查厂表选机座" if ok is None else "公称功率不足"),
	}
