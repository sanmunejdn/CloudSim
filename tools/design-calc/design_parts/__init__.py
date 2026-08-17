"""标准件库：检索 + instantiate → feature.compose JSON。"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path
from typing import Any

PARTS_ROOT = Path(__file__).resolve().parent.parent / "parts"


def _load_json(path: Path) -> dict[str, Any]:
	return json.loads(path.read_text(encoding="utf-8"))


def list_parts(root: Path | None = None) -> list[dict[str, Any]]:
	base = root or PARTS_ROOT
	out: list[dict[str, Any]] = []
	if not base.is_dir():
		return out
	for d in sorted(base.iterdir()):
		pj = d / "part.json"
		if not pj.is_file():
			continue
		meta = _load_json(pj)
		meta["_dir"] = str(d)
		out.append(meta)
	return out


def get_part(part_id: str, root: Path | None = None) -> dict[str, Any]:
	base = root or PARTS_ROOT
	d = base / part_id
	pj = d / "part.json"
	if not pj.is_file():
		raise FileNotFoundError(f"part not found: {part_id}")
	meta = _load_json(pj)
	meta["_dir"] = str(d)
	return meta


def search_parts(query: str, root: Path | None = None) -> list[dict[str, Any]]:
	q = query.strip().lower()
	hits: list[tuple[int, dict[str, Any]]] = []
	for p in list_parts(root):
		score = 0
		pid = str(p.get("id", "")).lower()
		if pid == q or pid.endswith(q):
			score += 10
		for kw in p.get("keywords") or []:
			k = str(kw).lower()
			if k and k in q:
				score += 5
			elif k and q in k:
				score += 2
		if "螺栓" in q and "bolt" in pid:
			score += 3
		if "螺母" in q and "nut" in pid:
			score += 3
		if "垫" in q and "washer" in pid:
			score += 3
		if ("销" in q or "pin" in q) and "pin." in pid:
			score += 3
		if "齿轮" in q and "gear." in pid:
			score += 3
		if score > 0:
			hits.append((score, p))
	hits.sort(key=lambda x: -x[0])
	return [h[1] for h in hits]


def _hex_vertex_radius(across_flats_mm: float) -> float:
	# 对边宽度 s → 外接圆半径
	return across_flats_mm / math.sqrt(3.0)


def _merge_spec(part: dict[str, Any], params: dict[str, Any]) -> dict[str, Any]:
	merged = dict(part.get("defaults") or {})
	merged.update({k: v for k, v in params.items() if v is not None})
	specs = part.get("specs") or []
	chosen = None
	if "thread" in merged:
		for s in specs:
			if str(s.get("thread")) == str(merged["thread"]):
				chosen = dict(s)
				break
	elif "d_mm" in merged and part["id"].startswith("pin."):
		for s in specs:
			if float(s.get("d_mm", -1)) == float(merged["d_mm"]):
				chosen = dict(s)
				break
	elif part["id"].startswith("gear."):
		# 精确匹配或用入参直接算
		for s in specs:
			if (
				float(s.get("m_mm", -1)) == float(merged.get("m_mm", -2))
				and int(s.get("Z", -1)) == int(merged.get("Z", -2))
			):
				chosen = dict(s)
				break
		if chosen is None:
			chosen = {}
	if chosen:
		for k, v in chosen.items():
			if k == "length_series_mm":
				continue
			merged.setdefault(k, v)
		series = chosen.get("length_series_mm")
		if series and "length_mm" in merged:
			L = float(merged["length_mm"])
			if L not in [float(x) for x in series]:
				# 允许非标长度，仅提示
				merged["_length_nonstandard"] = True
	return merged


def resolve_bind_vars(part: dict[str, Any], params: dict[str, Any]) -> dict[str, Any]:
	"""规格合并 + 模板占位符派生量。"""
	m = _merge_spec(part, params)
	pid = part["id"]
	if pid.startswith("fastener.hex_bolt") or pid.startswith("fastener.hex_nut"):
		s = float(m["s_mm"])
		m["head_r_mm"] = _hex_vertex_radius(s)
	if pid.startswith("pin."):
		m["radius_mm"] = float(m["d_mm"]) * 0.5
	if pid.startswith("gear."):
		mm = float(m["m_mm"])
		z = int(m["Z"])
		m["d_mm"] = mm * z
		m["da_mm"] = mm * (z + 2)
		m["radius_mm"] = m["da_mm"] * 0.5
		m.setdefault("b_mm", 10.0 * mm)
		m.setdefault("bore_mm", 0)
	return m


def _fill_template(obj: Any, vars_map: dict[str, Any]) -> Any:
	if isinstance(obj, dict):
		return {k: _fill_template(v, vars_map) for k, v in obj.items()}
	if isinstance(obj, list):
		return [_fill_template(v, vars_map) for v in obj]
	if isinstance(obj, str) and obj.startswith("{{") and obj.endswith("}}"):
		key = obj[2:-2].strip()
		if key not in vars_map:
			raise KeyError(f"missing template var: {key}")
		val = vars_map[key]
		if isinstance(val, bool):
			return val
		return val
	return obj


def instantiate(part_id: str, params: dict[str, Any] | None = None, root: Path | None = None) -> dict[str, Any]:
	"""part_id + params → feature.compose JSON。"""
	part = get_part(part_id, root)
	vars_map = resolve_bind_vars(part, params or {})
	if "hex_bolt" in part_id:
		d = float(vars_map["d_mm"])
		length = float(vars_map["length_mm"])
		k = float(vars_map["k_mm"])
		rh = float(vars_map["head_r_mm"])
		r = d * 0.5
		xyz: list[float] = []
		for x, y in ((0, 0), (r, 0), (r, length), (rh, length), (rh, length + k), (0, length + k), (0, 0)):
			xyz.extend((x, y, 0.0))
		return {
			"version": 2,
			"domain": "feature.compose",
			"meta": {
				"part_id": part_id,
				"blank_only": True,
				"model_fidelity": "blank",
				"head_style": "cylinder_approx",
				"params": {k: vars_map[k] for k in vars_map if not str(k).startswith("_")},
			},
			"steps": [
				{
					"id": "bolt",
					"api": "revolveSketchProfileToBrep",
					"args": {
						"mode": "boss",
						"profile_xyz_mm": xyz,
						"angle_deg": 360.0,
						"axis_dx": 0.0,
						"axis_dy": 1.0,
						"axis_dz": 0.0,
						"name": "HexBoltBlank",
					},
				}
			],
		}
	model_name = str(part.get("model_ref", "model.compose.json"))
	model_path = Path(part["_dir"]) / model_name
	tpl = _load_json(model_path)
	plan = _fill_template(tpl, vars_map)
	if not isinstance(plan, dict):
		raise RuntimeError("compose template root must be object")
	plan["domain"] = "feature.compose"
	plan["version"] = 2
	meta = plan.setdefault("meta", {})
	meta["part_id"] = part_id
	meta["params"] = {k: vars_map[k] for k in vars_map if not str(k).startswith("_")}
	meta["blank_only"] = True
	meta["model_fidelity"] = part.get("model_fidelity", "blank")
	return plan


_THREAD_RE = re.compile(r"\bM\s*(\d+)\b", re.I)
_LEN_RE = re.compile(r"(?:长|长度|L\s*[=:]?\s*|x|×)\s*(\d+(?:\.\d+)?)\s*mm?", re.I)
_MOD_RE = re.compile(r"(?:模数|m)\s*[=:]?\s*(\d+(?:\.\d+)?)", re.I)
_Z_RE = re.compile(r"(?:齿数|Z)\s*[=:]?\s*(\d+)", re.I)
_D_RE = re.compile(r"(?:直径|d)\s*[=:]?\s*(\d+(?:\.\d+)?)", re.I)


def parse_user_text_to_instantiate(text: str, root: Path | None = None) -> dict[str, Any]:
	"""口语 → {part_id, params, plan}；找不到则 ask 澄清结构。"""
	hits = search_parts(text, root)
	if not hits:
		return {
			"ok": False,
			"error": "未匹配到标准件",
			"hint": "可试：六角螺栓 M8×30、螺母 M8、垫圈 M8、销 d6 长20、齿轮 模数2 齿数20",
		}
	part = hits[0]
	params: dict[str, Any] = {}
	mt = _THREAD_RE.search(text)
	if mt:
		params["thread"] = f"M{mt.group(1)}"
	ln = _LEN_RE.search(text)
	if not ln:
		ln = re.search(r"[x×]\s*(\d+(?:\.\d+)?)", text)
	if ln:
		params["length_mm"] = float(ln.group(1))
	if part["id"].startswith("pin."):
		dm = _D_RE.search(text)
		if dm:
			params["d_mm"] = float(dm.group(1))
	if part["id"].startswith("gear."):
		mm = _MOD_RE.search(text)
		zz = _Z_RE.search(text)
		if mm:
			params["m_mm"] = float(mm.group(1))
		if zz:
			params["Z"] = int(zz.group(1))
		bm = re.search(r"(?:齿宽|b)\s*[=:]?\s*(\d+(?:\.\d+)?)", text, re.I)
		if bm:
			params["b_mm"] = float(bm.group(1))
	plan = instantiate(part["id"], params, root)
	return {"ok": True, "part_id": part["id"], "params": plan["meta"]["params"], "plan": plan}


def write_index(root: Path | None = None) -> Path:
	base = root or PARTS_ROOT
	items = []
	for p in list_parts(base):
		items.append(
			{
				"id": p["id"],
				"display_name": p.get("display_name"),
				"keywords": p.get("keywords"),
				"model_fidelity": p.get("model_fidelity"),
				"defaults": p.get("defaults"),
			}
		)
	out = base / "index.json"
	out.write_text(json.dumps({"version": 1, "parts": items}, ensure_ascii=False, indent=2), encoding="utf-8")
	return out
