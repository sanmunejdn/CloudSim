from __future__ import annotations

import json
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from design_parts import (  # noqa: E402
	instantiate,
	list_parts,
	parse_user_text_to_instantiate,
	search_parts,
	write_index,
)


class PartsTests(unittest.TestCase):
	def test_list_five(self):
		ids = [p["id"] for p in list_parts()]
		self.assertEqual(len(ids), 5)

	def test_bolt(self):
		plan = instantiate("fastener.hex_bolt_iso4017", {"thread": "M8", "length_mm": 30})
		self.assertEqual(plan["domain"], "feature.compose")
		self.assertEqual(len(plan["steps"]), 1)
		self.assertEqual(plan["steps"][0]["api"], "revolveSketchProfileToBrep")
		xyz = plan["steps"][0]["args"]["profile_xyz_mm"]
		self.assertGreaterEqual(len(xyz), 21)
		self.assertIn("cylinder_approx", plan["meta"].get("head_style", ""))

	def test_nut_washer_pin_gear(self):
		self.assertEqual(instantiate("fastener.hex_nut_iso4032", {"thread": "M8"})["domain"], "feature.compose")
		self.assertEqual(instantiate("fastener.plain_washer_iso7089", {"thread": "M10"})["domain"], "feature.compose")
		pin = instantiate("pin.cylindrical_iso2338", {"d_mm": 6, "length_mm": 20})
		self.assertAlmostEqual(float(pin["steps"][0]["args"]["length_mm"]), 3.0)
		g = instantiate("gear.spur_blank", {"m_mm": 2, "Z": 20, "b_mm": 20})
		self.assertAlmostEqual(float(g["steps"][0]["args"]["length_mm"]), 22.0)

	def test_search_parse(self):
		self.assertTrue(search_parts("六角螺栓"))
		r = parse_user_text_to_instantiate("创建六角螺栓 M8×30")
		self.assertTrue(r["ok"])
		self.assertEqual(r["part_id"], "fastener.hex_bolt_iso4017")

	def test_index(self):
		path = write_index()
		data = json.loads(path.read_text(encoding="utf-8"))
		self.assertEqual(len(data["parts"]), 5)


if __name__ == "__main__":
	unittest.main()
