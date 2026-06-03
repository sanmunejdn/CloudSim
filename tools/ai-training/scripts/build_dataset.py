#!/usr/bin/env python3
"""校验 domains/<id>/dataset.jsonl：合法 JSON + mesh.create / mesh.compose 输出 schema。"""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MIN_DIM = 0.1
MAX_DIM = 1e6
PRIMITIVES = frozenset({"box", "cylinder", "cone", "sphere"})
RECOGNITION_PRIMITIVES = frozenset({"box", "cylinder", "cone", "sphere", "unknown"})


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


def validate_geometry_recognize_output(out: dict, line_no: int) -> list[str]:
    errs = []
    prim = out.get("primitive")
    if prim not in RECOGNITION_PRIMITIVES:
        errs.append(f"line {line_no}: invalid primitive {prim!r}")
        return errs
    conf = out.get("confidence")
    if conf is not None and not isinstance(conf, (int, float)):
        errs.append(f"line {line_no}: confidence must be number")
    if prim == "unknown":
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


KNOWN_APIS = frozenset({"createPrimitiveMesh", "booleanMesh", "importFileIntoActiveDocument"})
TRAJECTORY_AXES = frozenset({"line", "surface", "ambiguous"})
TRAJECTORY_PIPELINES = frozenset({"weld_default", "glue_default", "grind_default"})


def validate_mesh_compose_output(out: dict, line_no: int) -> list[str]:
    errs = []
    if out.get("version") != 2:
        errs.append(f"line {line_no}: version must be 2")
    steps = out.get("steps")
    if not isinstance(steps, list) or not steps:
        errs.append(f"line {line_no}: steps[] required")
        return errs
    defined: set[str] = set()
    for si, step in enumerate(steps):
        if not isinstance(step, dict):
            errs.append(f"line {line_no} step {si}: must be object")
            continue
        api = step.get("api")
        if api not in KNOWN_APIS:
            errs.append(f"line {line_no} step {si}: unknown api {api!r}")
        args = step.get("args") if isinstance(step.get("args"), dict) else {}
        if api == "createPrimitiveMesh":
            sub = {
                "version": 1,
                "action": "create_mesh",
                "primitive": args.get("primitive", "box"),
                "dimensions_mm": args.get("dimensions_mm", {}),
            }
            errs.extend(validate_mesh_create_output(sub, line_no))
            pose = args.get("pose_mm")
            if pose is not None:
                if not isinstance(pose, dict):
                    errs.append(f"line {line_no} step {si}: pose_mm must be object")
                else:
                    for k in ("x", "y", "z"):
                        if k not in pose or not isinstance(pose[k], (int, float)):
                            errs.append(f"line {line_no} step {si}: pose_mm.{k} required")
        elif api == "booleanMesh":
            op = args.get("op", "difference")
            if op not in ("difference", "union", "intersection"):
                errs.append(f"line {line_no} step {si}: invalid op")
            for key in ("target", "tool"):
                ref = args.get(key, "")
                if not isinstance(ref, str) or not ref:
                    errs.append(f"line {line_no} step {si}: {key} required")
                elif ref.startswith("$"):
                    sid = ref[1:]
                    if sid not in defined:
                        errs.append(f"line {line_no} step {si}: {key} ref ${sid} not defined yet")
        sid = step.get("id")
        if isinstance(sid, str) and sid:
            defined.add(sid)
    return errs


def validate_trajectory_feature_output(out: dict, line_no: int) -> list[str]:
    errs = []
    if out.get("version", 1) != 1:
        errs.append(f"line {line_no}: version must be 1")
    axis = out.get("featureAxis")
    if axis not in TRAJECTORY_AXES:
        errs.append(f"line {line_no}: invalid featureAxis {axis!r}")
    if axis == "ambiguous" and not out.get("clarifyMessage"):
        errs.append(f"line {line_no}: ambiguous requires clarifyMessage")
    ids = out.get("selectedCandidateIds")
    if ids is not None and not isinstance(ids, list):
        errs.append(f"line {line_no}: selectedCandidateIds must be array")
    tpl = out.get("suggestedPipelineTemplate")
    if tpl is not None and tpl not in TRAJECTORY_PIPELINES:
        errs.append(f"line {line_no}: invalid suggestedPipelineTemplate {tpl!r}")
    feats = out.get("features")
    if feats is not None and not isinstance(feats, list):
        errs.append(f"line {line_no}: features must be array")
    return errs


def compose_op_from_output(out: dict) -> str:
    for step in out.get("steps", []):
        if isinstance(step, dict) and step.get("api") == "booleanMesh":
            args = step.get("args") if isinstance(step.get("args"), dict) else {}
            return args.get("op", "difference")
    return "create_only"


def main() -> int:
    domain = sys.argv[1] if len(sys.argv) > 1 else "mesh.create"
    path = ROOT / "domains" / domain / "dataset.jsonl"
    if not path.is_file():
        print(f"missing: {path}", file=sys.stderr)
        return 1
    errors: list[str] = []
    warnings: list[str] = []
    count = 0
    op_counts: dict[str, int] = {}
    instruction_seen: dict[str, int] = {}
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
        elif domain == "mesh.compose":
            errors.extend(validate_mesh_compose_output(out, i))
            op = compose_op_from_output(out)
            op_counts[op] = op_counts.get(op, 0) + 1
        elif domain == "geometry.recognize":
            errors.extend(validate_geometry_recognize_output(out, i))
            img = row.get("input", "")
            if isinstance(img, str) and img and img != "<image>":
                img_path = path.parent / img
                if not img_path.is_file():
                    errors.append(f"line {i}: missing image {img}")
        elif domain == "trajectory.feature":
            errors.extend(validate_trajectory_feature_output(out, i))
            inp = row.get("input")
            if inp is not None and not isinstance(inp, (str, dict)):
                errors.append(f"line {i}: input must be string or object")
        ins = row.get("instruction", "")
        if isinstance(ins, str) and ins:
            instruction_seen[ins] = instruction_seen.get(ins, 0) + 1
    for ins, n in instruction_seen.items():
        if n > 1:
            warnings.append(f"duplicate instruction ({n}x): {ins[:80]}")
    if errors:
        for e in errors:
            print(e, file=sys.stderr)
        return 1
    print(f"ok: {path} ({count} samples)")
    if domain == "mesh.compose" and op_counts:
        parts = ", ".join(f"{k}={v}" for k, v in sorted(op_counts.items()))
        print(f"  boolean ops: {parts}")
    for w in warnings:
        print(f"warning: {w}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
