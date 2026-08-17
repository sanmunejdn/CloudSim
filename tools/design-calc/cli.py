#!/usr/bin/env python3
"""design.calc CLI：计算并可选导出 feature.compose JSON。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from design_calc import (  # noqa: E402
	gear_pair_to_feature_compose,
	gear_rack,
	gear_rack_to_feature_compose,
	gear_spur_helical_shift,
	lookup_y_motor,
	motor_power,
	reducer_rated_power,
	worm_geometry,
	worm_to_feature_compose,
)


def main() -> int:
	p = argparse.ArgumentParser(description="CloudSim design.calc")
	p.add_argument("cmd", choices=["motor", "reducer", "y-motor", "gear", "rack", "worm"])
	p.add_argument("--json", type=str, help="输入 JSON 对象或文件路径")
	p.add_argument("--compose-out", type=str, help="写出 feature.compose JSON 路径")
	args = p.parse_args()

	payload: dict = {}
	if args.json:
		path = Path(args.json)
		if path.is_file():
			payload = json.loads(path.read_text(encoding="utf-8"))
		else:
			payload = json.loads(args.json)

	compose = None
	if args.cmd == "motor":
		out = motor_power(payload or {"mode": "force_speed", "Vw_m_s": 0.1, "F_N": 1000, "eta_w": 0.85})
	elif args.cmd == "reducer":
		out = reducer_rated_power(
			payload or {"KA": 1.5, "KS": 1.5, "P_kW": 300, "N_rpm": 1200, "n_rpm": 41, "P1_catalog_kW": 840}
		)
	elif args.cmd == "y-motor":
		out = lookup_y_motor(payload or {"power_kW": 1.5})
	elif args.cmd == "gear":
		out = gear_spur_helical_shift(
			payload or {"kind": "spur", "Z1": 20, "Z2": 40, "m_mm": 2, "A_mm": 60, "b_mm": 20}
		)
		compose = gear_pair_to_feature_compose(out)
	elif args.cmd == "rack":
		out = gear_rack(payload or {"Z1": 20, "m_mm": 2, "n1_rpm": 50})
		compose = gear_rack_to_feature_compose(out)
	elif args.cmd == "worm":
		out = worm_geometry(payload or {"m_mm": 2, "Z1": 1, "Z2": 40, "q": 10})
		compose = worm_to_feature_compose(out)
	else:
		return 2

	print(json.dumps(out, ensure_ascii=False, indent=2))
	if compose is not None:
		print("--- feature.compose ---")
		print(json.dumps(compose, ensure_ascii=False, indent=2))
		if args.compose_out:
			Path(args.compose_out).write_text(json.dumps(compose, ensure_ascii=False, indent=2), encoding="utf-8")
			print("wrote", args.compose_out)
	return 0


if __name__ == "__main__":
	raise SystemExit(main())
