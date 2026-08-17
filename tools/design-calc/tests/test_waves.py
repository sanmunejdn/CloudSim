from __future__ import annotations

import json
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from design_calc import (  # noqa: E402
	belt_v_length,
	chain_sprocket_pitch_diameter,
	gear_high_shift_dims,
	gear_pair_to_feature_compose,
	gear_rack,
	gear_rack_to_feature_compose,
	gear_spur_helical_shift,
	inertia_cylinder,
	key_strength,
	load_torque_ballscrew,
	lookup_gear_material,
	lookup_y_motor,
	motor_power,
	reducer_rated_power,
	worm_geometry,
	worm_to_feature_compose,
)


class WaveATests(unittest.TestCase):
	def test_motor_force_speed(self):
		r = motor_power({"mode": "force_speed", "Vw_m_s": 0.1, "F_N": 1000, "eta_w": 0.85})
		self.assertAlmostEqual(r["Pw_kW"], 0.1176470588, places=6)

	def test_motor_torque_speed(self):
		r = motor_power({"mode": "torque_speed", "Mw_Nm": 160, "nw_rpm": 15, "eta_w": 0.85})
		self.assertAlmostEqual(r["Pw_kW"], 0.29565753, places=5)

	def test_motor_md(self):
		r = motor_power({"mode": "motor_torque", "Pd_kW": 1.5, "nd_rpm": 25, "eta_d": 1})
		self.assertAlmostEqual(r["Md_Nm"], 573.0, places=3)

	def test_reducer(self):
		r = reducer_rated_power({"KA": 1.5, "KS": 1.5, "P_kW": 300, "N_rpm": 1200, "n_rpm": 41, "P1_catalog_kW": 840})
		self.assertAlmostEqual(r["P2m_kW"], 675.0, places=6)
		self.assertAlmostEqual(r["i"], 1200 / 41, places=6)
		self.assertTrue(r["pass"])

	def test_y_lookup(self):
		r = lookup_y_motor({"power_kW": 1.4, "prefer_2pole": True})
		self.assertTrue(r["ok"])
		self.assertEqual(r["selected"]["model"], "Y90S-2")

	def test_inertia(self):
		r = inertia_cylinder({"d0_mm": 500, "d1_mm": 0, "L_mm": 10, "rho_kg_m3": 7800, "e_mm": 50})
		self.assertGreater(r["mass_kg"], 0)
		self.assertGreater(r["Jx_kgm2"], r["J0_kgm2"])

	def test_load_ballscrew(self):
		r = load_torque_ballscrew({"F_N": 1000, "Pb_mm_rev": 10, "eta": 0.9, "i": 1})
		self.assertGreater(r["TL_Nm"], 0)


class WaveBTests(unittest.TestCase):
	def test_spur_no_shift(self):
		r = gear_spur_helical_shift({"kind": "spur", "Z1": 20, "Z2": 40, "m_mm": 2, "A_mm": 60, "b_mm": 20})
		self.assertAlmostEqual(r["A0_mm"], 60.0, places=6)
		self.assertAlmostEqual(r["pinion"]["d_mm"], 40.0, places=6)
		self.assertAlmostEqual(r["pinion"]["da_mm"], 44.0, places=6)

	def test_rack(self):
		r = gear_rack({"Z1": 20, "m_mm": 2, "n1_rpm": 50})
		self.assertAlmostEqual(r["pinion"]["d_mm"], 40.0, places=6)
		self.assertAlmostEqual(r["V_mm_s"], math.pi * 40 * 50 / 60.0, places=5)

	def test_high_shift(self):
		r = gear_high_shift_dims({"Z": 52, "m_mm": 2.5, "X": 0})
		self.assertAlmostEqual(r["d_mm"], 130.0, places=6)
		self.assertGreater(r["Wk_mm"], 0)

	def test_material(self):
		r = lookup_gear_material({"grade": "45"})
		self.assertTrue(r["ok"])
		self.assertGreaterEqual(len(r["matches"]), 1)


class WaveCTests(unittest.TestCase):
	def test_worm(self):
		r = worm_geometry({"m_mm": 2, "Z1": 1, "Z2": 40, "q": 10})
		self.assertAlmostEqual(r["A_mm"], 0.5 * (20 + 80), places=6)
		self.assertAlmostEqual(r["i"], 40.0, places=6)

	def test_key(self):
		r = key_strength({"T_Nm": 100, "d_mm": 40, "b_mm": 12, "h_mm": 8, "L_mm": 50})
		self.assertIn("ok", r)

	def test_belt_chain(self):
		b = belt_v_length({"a_mm": 500, "D1_mm": 100, "D2_mm": 200})
		self.assertGreater(b["L_mm"], 1000)
		c = chain_sprocket_pitch_diameter({"p_mm": 12.7, "Z": 20})
		self.assertAlmostEqual(c["d_mm"], 12.7 / math.sin(math.pi / 20), places=6)


class ComposeTests(unittest.TestCase):
	def test_gear_compose(self):
		g = gear_spur_helical_shift({"kind": "spur", "Z1": 20, "Z2": 40, "m_mm": 2, "A_mm": 60, "b_mm": 20})
		plan = gear_pair_to_feature_compose(g)
		self.assertEqual(plan["domain"], "feature.compose")
		self.assertEqual(len(plan["steps"]), 2)
		self.assertEqual(plan["steps"][0]["api"], "revolveSketchProfileToBrep")

	def test_rack_worm_compose(self):
		rack = gear_rack({"Z1": 20, "m_mm": 2})
		self.assertEqual(gear_rack_to_feature_compose(rack)["domain"], "feature.compose")
		w = worm_geometry({"m_mm": 2, "Z2": 40})
		plan = worm_to_feature_compose(w)
		self.assertEqual(len(plan["steps"]), 2)
		# 可序列化
		json.dumps(plan)


if __name__ == "__main__":
	unittest.main()
