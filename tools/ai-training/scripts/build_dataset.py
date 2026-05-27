#!/usr/bin/env python3
"""校验 domains/<id>/dataset.jsonl：合法 JSON + mesh.create 输出 schema。"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MIN_DIM = 0.1
MAX_DIM = 1e6
PRIMITIVES = frozenset({"box", "cylinder", "cone", "sphere"})


def _num(v):
    if not isinstance(v, (int, float)):
        return None
    x = float(v)
    if x < MIN_DIM or x > MAX_DIM:
        return None
    return x


def validate_mesh_create_output(out: dict, line_no: int) -> list[str]:
    errs = []
    if out.get("version", 1) != 1:
        errs.append(f"line {line_no}: version must be 1")
    if out.get("action") != "create_mesh":
        errs.append(f"line {line_no}: action must be create_mesh")
    prim = out.get("primitive")
    if prim not in PRIMITIVES:
        errs.append(f"line {line_no}: invalid primitive {prim!r}")
        return errs
    dims = out.get("dimensions_mm")
    if not isinstance(dims, dict):
        errs.append(f"line {line_no}: dimensions_mm must be object")
        return errs
    if prim == "box":
        for k in ("length", "width", "height"):
            if _num(dims.get(k)) is None:
                errs.append(f"line {line_no}: box missing/invalid {k}")
    elif prim in ("cylinder", "cone"):
        if _num(dims.get("radius")) is None:
            errs.append(f"line {line_no}: {prim} missing/invalid radius")
        if _num(dims.get("height")) is None:
            errs.append(f"line {line_no}: {prim} missing/invalid height")
    elif prim == "sphere":
        r = _num(dims.get("radius"))
        d = _num(dims.get("diameter"))
        if r is None and d is None:
            errs.append(f"line {line_no}: sphere needs radius or diameter")
    return errs


def main() -> int:
    domain = sys.argv[1] if len(sys.argv) > 1 else "mesh.create"
    path = ROOT / "domains" / domain / "dataset.jsonl"
    if not path.is_file():
        print(f"missing: {path}", file=sys.stderr)
        return 1
    errors: list[str] = []
    count = 0
    for i, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        count += 1
        try:
            row = json.loads(line)
        except json.JSONDecodeError as e:
            errors.append(f"line {i}: JSON parse error: {e}")
            continue
        if not isinstance(row, dict):
            errors.append(f"line {i}: row must be object")
            continue
        if "instruction" not in row or "output" not in row:
            errors.append(f"line {i}: need instruction and output")
            continue
        try:
            out = json.loads(row["output"]) if isinstance(row["output"], str) else row["output"]
        except json.JSONDecodeError as e:
            errors.append(f"line {i}: output JSON: {e}")
            continue
        if domain == "mesh.create":
            errors.extend(validate_mesh_create_output(out, i))
    if errors:
        for e in errors:
            print(e, file=sys.stderr)
        return 1
    print(f"ok: {path} ({count} samples)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
