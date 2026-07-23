# -*- coding: utf-8 -*-
"""配天 AIR：Canonical v1 → .arl + _data.arl"""

from __future__ import annotations

import os
from datetime import datetime

from canonical_v1 import (
    condition_text,
    duration_sec,
    ensure_output_ext,
    export_entry,
    io_port,
    load_canonical_v1,
    motion_blend,
    motion_speed,
    pose_mm_deg,
    tool_frame_name,
    user_frame_name,
    walk_nodes,
)


def _tool_token(name: str) -> str:
    n = (name or "tool0").lower()
    return n if n.startswith("$") else f"${n}"


def _wobj_token(name: str) -> str:
    n = (name or "wobj0").lower()
    return n if n.startswith("$") else f"${n}"


def convert(OutPutPath: str, CanonicalPath: str, comment: str = "") -> None:
    doc = load_canonical_v1(CanonicalPath)
    out = ensure_output_ext(OutPutPath, ".arl")
    comment = comment or f"From Canonical {os.path.basename(CanonicalPath)} @ {datetime.now():%Y-%m-%d %H:%M:%S}"

    points: dict[int, dict] = {}
    logic: list[str] = []
    point_idx = 0

    for event, node in walk_nodes(list(doc.get("instructions") or [])):
        t = str(node.get("type", "")).lower()
        if event == "if_enter":
            logic.append(f"  // IF {condition_text(node)}")
            continue
        if event == "if_else":
            logic.append("  // ELSE")
            continue
        if event == "if_leave":
            logic.append("  // ENDIF")
            continue
        if event == "while_enter":
            logic.append(f"  // WHILE {condition_text(node)}")
            continue
        if event == "while_leave":
            logic.append("  // ENDWHILE")
            continue
        if event != "node":
            continue

        if t in ("ptp", "line"):
            point_idx += 1
            pose = pose_mm_deg(node)
            points[point_idx] = pose
            speed = int(motion_speed(node, 100 if t == "ptp" else 500))
            br = int(motion_blend(node, 100))
            tool = _tool_token(tool_frame_name(node))
            wobj = _wobj_token(user_frame_name(node))
            move = "MoveJ" if t == "ptp" else "MoveL"
            logic.append(f"  {move} p:p{point_idx},v{speed},z{br},t:{tool},w:{wobj}")
        elif t == "wait":
            logic.append(f"  Delay {duration_sec(node)}")
        elif t in ("set_do", "set_ao"):
            logic.append(f"  // Set {io_port(node)}")
        else:
            logic.append(f"  // skip {t}")

    data_lines = [f"// {comment} - Data"]
    for idx, pose in sorted(points.items()):
        data_lines.append(
            f"p{idx} = {{{pose['x']:.3f},{pose['y']:.3f},{pose['z']:.3f},"
            f"{pose['rx']:.3f},{pose['ry']:.3f},{pose['rz']:.3f}}}"
        )

    base_dir = os.path.dirname(out) or "."
    base_name = os.path.splitext(os.path.basename(out))[0]
    data_path = os.path.join(base_dir, f"{base_name}_data.arl")
    prog = [f"// {comment}", "func void main()", "  init()"] + logic + ["endfunc", ""]

    os.makedirs(base_dir, exist_ok=True)
    with open(data_path, "w", encoding="utf-8") as f:
        f.write("\n".join(data_lines) + "\n")
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(prog) + "\n")


ExportScript, run = export_entry(convert)
