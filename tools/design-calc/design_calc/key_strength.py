from __future__ import annotations

from typing import Any


def key_strength(payload: dict[str, Any]) -> dict[str, Any]:
	"""平键挤压与剪切强度校核（静载常用式）。

	σ_p = 4T / (d h L) ；τ = 2T / (d b L)
	T:N·mm，尺寸 mm → σ、τ 为 N/mm²
	"""
	t_nm = float(payload["T_Nm"])
	d = float(payload["d_mm"])
	b = float(payload["b_mm"])
	h = float(payload["h_mm"])
	length = float(payload["L_mm"])
	sigma_p_allow = float(payload.get("sigma_p_allow", 100.0))
	tau_allow = float(payload.get("tau_allow", 60.0))
	t = t_nm * 1000.0
	if min(d, b, h, length) <= 0:
		raise ValueError("dimensions must be > 0")
	sigma_p = 4.0 * t / (d * h * length)
	tau = 2.0 * t / (d * b * length)
	return {
		"sigma_p_N_mm2": sigma_p,
		"tau_N_mm2": tau,
		"sigma_p_ok": sigma_p <= sigma_p_allow + 1e-9,
		"tau_ok": tau <= tau_allow + 1e-9,
		"ok": sigma_p <= sigma_p_allow + 1e-9 and tau <= tau_allow + 1e-9,
	}
