#!/usr/bin/env python3
"""校验 domains/<id>/dataset.jsonl 每行是否为合法 JSON。"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    domain = sys.argv[1] if len(sys.argv) > 1 else "mesh.create"
    path = ROOT / "domains" / domain / "dataset.jsonl"
    if not path.is_file():
        print(f"missing: {path}", file=sys.stderr)
        return 1
    for i, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        json.loads(line)
    print(f"ok: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
