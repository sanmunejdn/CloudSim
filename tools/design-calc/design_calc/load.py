from __future__ import annotations

import math
from typing import Any


def load_torque_ballscrew(payload: dict[str, Any]) -> dict[str, Any]:
	"""滚珠丝杠驱动负载转矩（源表「负载转矩计算」子集）。

	TL ≈ (F + μ0*F0) * Pb / (2π η) / i   （N·m，Pb 为 m/rev）
	"""
	f = float(payload["F_N"])
	f0 = float(payload.get("F0_N", f / 3.0))
	mu0 = float(payload.get("mu0", 0.2))
	eta = float(payload.get("eta", 0.9))
	i = float(payload.get("i", 1.0))
	pb_mm = float(payload.get("Pb_mm_rev", payload.get("Pb_mm", 10.0)))
	pb = pb_mm / 1000.0
	if eta <= 0 or i <= 0:
		raise ValueError("eta and i must be > 0")
	tl = (f + mu0 * f0) * pb / (2.0 * math.pi * eta) / i
	return {
		"mechanism": "ballscrew",
		"TL_Nm": tl,
		"Pb_m_rev": pb,
		"inputs": {"F_N": f, "F0_N": f0, "mu0": mu0, "eta": eta, "i": i},
	}
