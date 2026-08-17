from __future__ import annotations

import csv
import math
from functools import lru_cache
from pathlib import Path
from typing import Any

_TABLE = Path(__file__).resolve().parent.parent / "tables" / "y_series_motors.csv"


@lru_cache(maxsize=1)
def _y_rows() -> list[dict[str, str]]:
	with _TABLE.open(encoding="utf-8-sig", newline="") as f:
		return list(csv.DictReader(f))


def lookup_y_motor(payload: dict[str, Any]) -> dict[str, Any]:
	"""按功率需求选最小满足的 Y 系列电机。"""
	need_kw = float(payload["power_kW"])
	min_rpm = float(payload.get("min_speed_rpm", 0.0))
	prefer_2pole = bool(payload.get("prefer_2pole", False))
	cands: list[dict[str, Any]] = []
	for r in _y_rows():
		try:
			p = float(r["power_kW"])
			n = float(r["speed_rpm"])
		except (KeyError, ValueError):
			continue
		if p + 1e-9 < need_kw:
			continue
		if n + 1e-9 < min_rpm:
			continue
		model = str(r.get("model", ""))
		if prefer_2pole and not model.endswith("-2"):
			continue
		cands.append(
			{
				"model": model,
				"power_kW": p,
				"speed_rpm": n,
				"current_A": float(r.get("current_A") or 0),
				"efficiency_pct": float(r.get("efficiency_pct") or 0),
				"power_factor": float(r.get("power_factor") or 0),
			}
		)
	cands.sort(key=lambda x: (x["power_kW"], -x["speed_rpm"]))
	if not cands:
		return {"ok": False, "error": "no motor matches", "candidates": []}
	return {"ok": True, "selected": cands[0], "candidates": cands[:5]}


def motor_power(payload: dict[str, Any]) -> dict[str, Any]:
	"""电机功率三套公式（对齐源表「电机功率确定程序」样例）。"""
	mode = str(payload.get("mode", "force_speed"))
	if mode == "force_speed":
		vw = float(payload["Vw_m_s"])
		f = float(payload["F_N"])
		eta = float(payload.get("eta_w", 0.85))
		if eta <= 0:
			raise ValueError("eta_w must be > 0")
		pw = f * vw / (1000.0 * eta)
		return {"mode": mode, "Pw_kW": pw, "unit": "kW"}
	if mode == "torque_speed":
		mw = float(payload["Mw_Nm"])
		nw = float(payload["nw_rpm"])
		eta = float(payload.get("eta_w", 0.85))
		if eta <= 0:
			raise ValueError("eta_w must be > 0")
		pw = mw * nw / (9550.0 * eta)
		return {"mode": mode, "Pw_kW": pw, "unit": "kW"}
	if mode == "motor_torque":
		pd = float(payload["Pd_kW"])
		nd = float(payload["nd_rpm"])
		eta_d = float(payload.get("eta_d", 1.0))
		if nd <= 0:
			raise ValueError("nd_rpm must be > 0")
		# 源表样例：Md = 9550*Pd/nd（eta_d=1）；有传动效率时按 Md/eta_d
		md = 9550.0 * pd / nd
		if eta_d > 0 and not math.isclose(eta_d, 1.0):
			md = md / eta_d
		return {"mode": mode, "Md_Nm": md, "Md_kgfm": md / 10.0, "unit": "N.m"}
	raise ValueError(f"unknown mode: {mode}")
