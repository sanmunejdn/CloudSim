#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from design_parts import instantiate, list_parts, parse_user_text_to_instantiate, search_parts, write_index  # noqa: E402


def main() -> int:
	p = argparse.ArgumentParser(description="design.parts instantiate / search")
	sub = p.add_subparsers(dest="cmd", required=True)

	s = sub.add_parser("list")
	s = sub.add_parser("index")
	s = sub.add_parser("search")
	s.add_argument("query")
	s = sub.add_parser("instantiate")
	s.add_argument("part_id")
	s.add_argument("--params", default="{}", help="JSON object")
	s.add_argument("-o", "--out")
	s = sub.add_parser("parse")
	s.add_argument("text")
	s.add_argument("-o", "--out")

	args = p.parse_args()
	if args.cmd == "list":
		print(json.dumps([x["id"] for x in list_parts()], ensure_ascii=False, indent=2))
		return 0
	if args.cmd == "index":
		path = write_index()
		print(path)
		return 0
	if args.cmd == "search":
		print(json.dumps([{"id": x["id"], "name": x.get("display_name")} for x in search_parts(args.query)], ensure_ascii=False, indent=2))
		return 0
	if args.cmd == "instantiate":
		plan = instantiate(args.part_id, json.loads(args.params))
		text = json.dumps(plan, ensure_ascii=False, indent=2)
		print(text)
		if args.out:
			Path(args.out).write_text(text, encoding="utf-8")
		return 0
	if args.cmd == "parse":
		r = parse_user_text_to_instantiate(args.text)
		text = json.dumps(r, ensure_ascii=False, indent=2)
		print(text)
		if args.out and r.get("ok") and r.get("plan"):
			Path(args.out).write_text(json.dumps(r["plan"], ensure_ascii=False, indent=2), encoding="utf-8")
		return 0 if r.get("ok") else 1
	return 2


if __name__ == "__main__":
	raise SystemExit(main())
