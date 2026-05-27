#!/usr/bin/env python3
"""按 domain 从 full_api_catalog.json 导出 catalog 切片。"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CATALOG = ROOT / "catalog" / "full_api_catalog.json"


def main() -> int:
    domain = sys.argv[1] if len(sys.argv) > 1 else "mesh.create"
    data = json.loads(CATALOG.read_text(encoding="utf-8"))
    apis = [a for a in data.get("apis", []) if domain in a.get("domains", [])]
    slice_doc = {"version": data.get("version", 1), "domain": domain, "apis": apis}
    print(json.dumps(slice_doc, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
