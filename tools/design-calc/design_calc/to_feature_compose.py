from __future__ import annotations

from typing import Any


def _revolve_cylinder_step(
	step_id: str,
	*,
	outer_r_mm: float,
	height_mm: float,
	name: str,
	inner_r_mm: float = 0.0,
) -> dict[str, Any]:
	"""矩形截面绕 Y 轴 360° → 圆柱毛坯（与 AI「旋转圆柱」：length=半径,width=高）。"""
	args: dict[str, Any] = {
		"mode": "boss",
		"profile": "rectangle",
		"length_mm": max(outer_r_mm, 0.1),
		"width_mm": height_mm,
		"angle_deg": 360.0,
		"axis_dx": 0.0,
		"axis_dy": 1.0,
		"axis_dz": 0.0,
		"name": name,
	}
	# 有内孔时用外径矩形再 pocket 圆孔由后续扩展；V1 实心毛坯
	if inner_r_mm > 0:
		args["note"] = f"blank_od={outer_r_mm*2};bore={inner_r_mm*2};teeth_not_modeled"
	return {"id": step_id, "api": "revolveSketchProfileToBrep", "args": args}


def _extrude_box_step(step_id: str, *, L: float, W: float, H: float, name: str) -> dict[str, Any]:
	return {
		"id": step_id,
		"api": "extrudeSketchProfileToBrep",
		"args": {
			"mode": "pad",
			"profile": "rectangle",
			"length_mm": L,
			"width_mm": W,
			"extrude_mm": H,
			"name": name,
		},
	}


def gear_pair_to_feature_compose(gear_result: dict[str, Any], *, bore1_mm: float = 0.0, bore2_mm: float = 0.0) -> dict[str, Any]:
	"""齿轮副计算结果 → feature.compose（毛坯，无齿廓）。"""
	p = gear_result["pinion"]
	g = gear_result["gear"]
	b = float(gear_result.get("b_mm", 10.0))
	meta = {
		"design_calc": "gear.spur_helical_shift",
		"A_mm": gear_result.get("A_mm"),
		"m_mm": gear_result.get("mn_mm") or gear_result.get("m_mm"),
		"blank_only": True,
	}
	return {
		"version": 2,
		"domain": "feature.compose",
		"meta": meta,
		"steps": [
			_revolve_cylinder_step(
				"pinion_blank",
				outer_r_mm=float(p["da_mm"]) * 0.5,
				height_mm=b,
				name="PinionBlank",
				inner_r_mm=bore1_mm * 0.5,
			),
			_revolve_cylinder_step(
				"gear_blank",
				outer_r_mm=float(g["da_mm"]) * 0.5,
				height_mm=b,
				name="GearBlank",
				inner_r_mm=bore2_mm * 0.5,
			),
		],
	}


def gear_rack_to_feature_compose(rack_result: dict[str, Any]) -> dict[str, Any]:
	p = rack_result["pinion"]
	r = rack_result["rack"]
	b = float(rack_result.get("b_mm", 10.0))
	return {
		"version": 2,
		"domain": "feature.compose",
		"meta": {"design_calc": "gear.rack", "blank_only": True},
		"steps": [
			_revolve_cylinder_step(
				"pinion_blank",
				outer_r_mm=float(p["da_mm"]) * 0.5,
				height_mm=b,
				name="RackPinionBlank",
			),
			_extrude_box_step(
				"rack_blank",
				L=float(r["length_mm"]),
				W=float(r["h_mm"]),
				H=float(r.get("thickness_mm", b)),
				name="RackBlank",
			),
		],
	}


def worm_to_feature_compose(worm_result: dict[str, Any]) -> dict[str, Any]:
	w = worm_result["worm"]
	wh = worm_result["wheel"]
	return {
		"version": 2,
		"domain": "feature.compose",
		"meta": {"design_calc": "worm.geometry", "blank_only": True, "A_mm": worm_result.get("A_mm")},
		"steps": [
			_revolve_cylinder_step(
				"worm_blank",
				outer_r_mm=float(w["da_mm"]) * 0.5,
				height_mm=float(w["length_suggest_mm"]),
				name="WormBlank",
			),
			_revolve_cylinder_step(
				"worm_wheel_blank",
				outer_r_mm=float(wh["da_mm"]) * 0.5,
				height_mm=float(wh["b_mm"]),
				name="WormWheelBlank",
			),
		],
	}
