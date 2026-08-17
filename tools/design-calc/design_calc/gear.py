from __future__ import annotations

import csv
import math
from functools import lru_cache
from pathlib import Path
from typing import Any

_MAT = Path(__file__).resolve().parent.parent / "tables" / "gear_materials_clean.csv"


def _rad(deg: float) -> float:
	return deg * math.pi / 180.0


def gear_spur_helical_shift(payload: dict[str, Any]) -> dict[str, Any]:
	"""外啮合变位圆柱齿轮几何（直/斜）。

	直齿：m；斜齿：法向模数 mn、螺旋角 β。中心距 A 可给，用于求总变位。
	"""
	kind = str(payload.get("kind", "spur"))  # spur | helical
	z1 = int(payload["Z1"])
	z2 = int(payload["Z2"])
	ha_star = float(payload.get("ha_star", 1.0))
	c_star = float(payload.get("c_star", 0.25))
	alpha_deg = float(payload.get("alpha_deg", 20.0))
	alpha = _rad(alpha_deg)
	b = float(payload.get("b_mm", 0.0))

	if kind == "helical":
		mn = float(payload.get("mn_mm", payload.get("m_mm", payload.get("mf", 0.0))))
		beta_deg = float(payload.get("beta_deg", payload.get("beta", 0.0)))
		beta = _rad(beta_deg)
		mt = mn / math.cos(beta)
		m = mt
	else:
		m = float(payload.get("m_mm", payload.get("m", 0.0)))
		mn = m
		beta_deg = 0.0
		beta = 0.0
		mt = m

	if m <= 0 or z1 <= 0 or z2 <= 0:
		raise ValueError("m/Z invalid")

	a0 = 0.5 * mt * (z1 + z2)
	a = float(payload["A_mm"]) if "A_mm" in payload or "A" in payload else a0
	if "A" in payload and "A_mm" not in payload:
		a = float(payload["A"])

	# 中心距变动系数 y；简化 inv 求解：无变位时 α'=α
	y = (a - a0) / mt
	# 总变位系数近似：ξΣ ≈ y + (标准齿高修正忽略时用手册曲线；V1 用无侧隙近似)
	# 当 A≈A0：ξΣ=0；否则用 ξΣ = (z1+z2)/2 * (invα'-invα)/tanα 需 α'
	# V1：若 |y|<1e-6 则 0，否则 ξΣ = y（工程常用初值，标注 approximate）
	xi_sum = 0.0 if abs(y) < 1e-9 else y
	xi1 = float(payload.get("xi1", xi_sum * z2 / (z1 + z2) if (z1 + z2) else 0.0))
	xi2 = float(payload.get("xi2", xi_sum - xi1))

	def wheel(z: int, xi: float) -> dict[str, float]:
		d = mt * z
		ha = (ha_star + xi) * mn if kind == "helical" else (ha_star + xi) * m
		hf = (ha_star + c_star - xi) * (mn if kind == "helical" else m)
		# 斜齿齿高按法向模数
		if kind != "helical":
			ha = (ha_star + xi) * m
			hf = (ha_star + c_star - xi) * m
		da = d + 2.0 * ha
		df = d - 2.0 * hf
		h = ha + hf
		return {"d_mm": d, "ha_mm": ha, "hf_mm": hf, "h_mm": h, "da_mm": da, "df_mm": df, "xi": xi}

	w1 = wheel(z1, xi1)
	w2 = wheel(z2, xi2)
	if b <= 0:
		b = 10.0 * mn

	return {
		"kind": kind,
		"m_mm": m,
		"mn_mm": mn,
		"mt_mm": mt,
		"beta_deg": beta_deg,
		"alpha_deg": alpha_deg,
		"A0_mm": a0,
		"A_mm": a,
		"y": y,
		"xi_sum": xi_sum,
		"xi_sum_approx": True,
		"b_mm": b,
		"pinion": {"Z": z1, **w1},
		"gear": {"Z": z2, **w2},
	}


def gear_rack(payload: dict[str, Any]) -> dict[str, Any]:
	"""齿轮齿条几何（直齿为主）。"""
	z1 = int(payload["Z1"])
	m = float(payload["m_mm"])
	ha_star = float(payload.get("ha_star", 1.0))
	c_star = float(payload.get("c_star", 0.25))
	x = float(payload.get("X", 0.0))
	n1 = float(payload.get("n1_rpm", 0.0))
	beta_deg = float(payload.get("beta_deg", 0.0))
	beta = _rad(beta_deg)
	d1 = m * z1 if abs(beta) < 1e-12 else (m * z1 / math.cos(beta))
	ha1 = (ha_star + x) * m
	hf1 = (ha_star + c_star - x) * m
	ha2 = ha_star * m
	hf2 = (ha_star + c_star) * m
	da1 = d1 + 2.0 * ha1
	df1 = d1 - 2.0 * hf1
	v = math.pi * d1 * n1 / 60.0 if n1 else 0.0
	b = float(payload.get("b_mm", 10.0 * m))
	rack_length = float(payload.get("rack_length_mm", 200.0))
	return {
		"m_mm": m,
		"beta_deg": beta_deg,
		"pinion": {
			"Z": z1,
			"d_mm": d1,
			"ha_mm": ha1,
			"hf_mm": hf1,
			"h_mm": ha1 + hf1,
			"da_mm": da1,
			"df_mm": df1,
			"X": x,
		},
		"rack": {"ha_mm": ha2, "hf_mm": hf2, "h_mm": ha2 + hf2, "length_mm": rack_length, "thickness_mm": b},
		"V_mm_s": v,
		"b_mm": b,
	}


def gear_high_shift_dims(payload: dict[str, Any]) -> dict[str, Any]:
	"""高变位单轮主要尺寸 + 公法线长度（α=20° 常用）。"""
	z = int(payload["Z"])
	m = float(payload["m_mm"])
	x = float(payload.get("X", 0.0))
	alpha_deg = float(payload.get("alpha_deg", 20.0))
	beta_deg = float(payload.get("beta_deg", 0.0))
	alpha = _rad(alpha_deg)
	beta = _rad(beta_deg)
	mn = m
	mt = mn / math.cos(beta) if abs(math.cos(beta)) > 1e-12 else mn
	d = mt * z
	ha = (1.0 + x) * mn
	hf = (1.0 + 0.25 - x) * mn
	da = d + 2.0 * ha
	df = d - 2.0 * hf
	# 跨齿数近似 k ≈ α/180*Z + 0.5
	k = int(round(alpha_deg / 180.0 * z + 0.5 + (2.0 * x / math.tan(alpha)) / math.pi))
	k = max(2, k)
	inv_a = math.tan(alpha) - alpha
	wk = mn * math.cos(alpha) * (math.pi * (k - 0.5) + z * inv_a + 2.0 * x * math.tan(alpha))
	return {
		"Z": z,
		"m_mm": m,
		"X": x,
		"k": k,
		"d_mm": d,
		"da_mm": da,
		"df_mm": df,
		"h_mm": ha + hf,
		"Wk_mm": wk,
		"b_suggest_mm": float(payload.get("b_mm", 10.0 * m)),
	}


@lru_cache(maxsize=1)
def _mat_rows() -> list[dict[str, str]]:
	if not _MAT.exists():
		return []
	with _MAT.open(encoding="utf-8-sig", newline="") as f:
		return list(csv.DictReader(f))


def lookup_gear_material(payload: dict[str, Any]) -> dict[str, Any]:
	grade = str(payload.get("grade", payload.get("材料牌号", ""))).strip()
	rows = [r for r in _mat_rows() if r.get("grade") == grade]
	if not rows:
		return {"ok": False, "error": f"material not found: {grade}", "matches": []}
	return {"ok": True, "matches": rows}
