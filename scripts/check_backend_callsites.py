# -*- coding: utf-8 -*-
"""backend() 穿透棘轮：只许减不许增。

用法（CloudSim 根）:
  python scripts/check_backend_callsites.py           # 对照基线，超标失败
  python scripts/check_backend_callsites.py --update  # 仅当计数下降时更新基线
  python scripts/check_backend_callsites.py --json
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
BASELINE = ROOT / "scripts" / "backend_callsites_baseline.json"

# Host 内部可保留；仍计入总数以便监控
CALL_RE = re.compile(r"(?:\.|->)backend\s*\(")
INCLUDE_RE = re.compile(r'#\s*include\s*[<"]BackendDataManager\.h[>"]')

# Widget 侧禁止新增 BackendDataManager.h（存量白名单相对路径）
WIDGET_BDM_ALLOW = {
	"src/UI/Widget/inc/DocumentPage.h",
	"src/UI/Widget/source/DocumentPage.cpp",
	"src/UI/Widget/source/MainWindowFileImport.cpp",
	"src/UI/Widget/source/MainWindowInstructionPropertyUiHost.cpp",
	"src/UI/Widget/source/MainWindowRobotHost.cpp",
}


def scan() -> dict:
	by_file: dict[str, int] = defaultdict(int)
	bdm_includes: list[str] = []
	for path in SRC.rglob("*"):
		if path.suffix.lower() not in {".cpp", ".h", ".hpp", ".cxx", ".cc"}:
			continue
		rel = path.relative_to(ROOT).as_posix()
		try:
			text = path.read_text(encoding="utf-8", errors="replace")
		except OSError:
			continue
		n = len(CALL_RE.findall(text))
		if n:
			by_file[rel] = n
		if INCLUDE_RE.search(text) and rel.startswith("src/UI/Widget/"):
			if rel not in WIDGET_BDM_ALLOW:
				bdm_includes.append(rel)
	total = sum(by_file.values())
	return {
		"total": total,
		"by_file": dict(sorted(by_file.items())),
		"forbidden_widget_bdm_includes": sorted(bdm_includes),
	}


def main() -> int:
	ap = argparse.ArgumentParser()
	ap.add_argument("--update", action="store_true")
	ap.add_argument("--json", action="store_true")
	args = ap.parse_args()
	cur = scan()
	if args.json:
		print(json.dumps(cur, ensure_ascii=False, indent=2))
	if args.update:
		if BASELINE.exists():
			old = json.loads(BASELINE.read_text(encoding="utf-8"))
			if cur["total"] > old.get("total", 0):
				print(f"REFUSE update: total {cur['total']} > baseline {old.get('total')}", file=sys.stderr)
				return 1
		BASELINE.write_text(json.dumps(cur, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
		print(f"UPDATED baseline total={cur['total']}")
		return 0
	if not BASELINE.exists():
		BASELINE.write_text(json.dumps(cur, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
		print(f"CREATED baseline total={cur['total']}")
		return 0
	old = json.loads(BASELINE.read_text(encoding="utf-8"))
	ok = True
	if cur["total"] > old.get("total", 0):
		print(f"FAIL: backend() calls {cur['total']} > baseline {old['total']}", file=sys.stderr)
		ok = False
	else:
		print(f"OK: backend() calls {cur['total']} <= baseline {old.get('total')}")
	if cur["forbidden_widget_bdm_includes"]:
		print("FAIL: Widget 新增 BackendDataManager.h include:", file=sys.stderr)
		for f in cur["forbidden_widget_bdm_includes"]:
			print(f"  {f}", file=sys.stderr)
		ok = False
	return 0 if ok else 1


if __name__ == "__main__":
	sys.exit(main())
