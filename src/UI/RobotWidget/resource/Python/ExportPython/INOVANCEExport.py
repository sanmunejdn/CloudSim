# -*- coding: utf-8 -*-
"""汇川 INOVANCE：Canonical v1 → .pro"""

from __future__ import annotations

import os
import re
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
    program_name_from_doc,
    safe_module_name,
    tool_frame_name,
    user_frame_name,
    walk_nodes,
)


def _num(s: str) -> int:
    m = re.search(r"\d+", s or "")
    return int(m.group()) if m else 0


def convert(OutPutPath: str, CanonicalPath: str, comment: str = "") -> None:
    doc = load_canonical_v1(CanonicalPath)
    name = safe_module_name(OutPutPath, program_name_from_doc(doc))
    out = ensure_output_ext(OutPutPath, ".pro")
    comment = comment or f"From Canonical {os.path.basename(CanonicalPath)} @ {datetime.now():%Y-%m-%d %H:%M:%S}"

    points: dict[int, dict] = {}
    instr: list[str] = []
    point_idx = -1

    for event, node in walk_nodes(doc.get("instructions") or []):
        t = str(node.get("type", "")).lower()
        if event == "if_enter":
            instr.append(f"    // IF {condition_text(node)}")
            continue
        if event == "if_else":
            instr.append("    // ELSE")
            continue
        if event == "if_leave":
            instr.append("    // ENDIF")
            continue
        if event == "while_enter":
            instr.append(f"    // WHILE {condition_text(node)}")
            continue
        if event == "while_leave":
            instr.append("    // ENDWHILE")
            continue
        if event != "node":
            continue

        if t in ("ptp", "line"):
            point_idx += 1
            pose = pose_mm_deg(node)
            points[point_idx] = {
                **pose,
                "utool": _num(tool_frame_name(node)),
                "uframe": _num(user_frame_name(node)),
            }
            speed = int(motion_speed(node, 100 if t == "ptp" else 500))
            br = int(motion_blend(node, 100))
            code = "Movj" if t == "ptp" else "Movl"
            p = points[point_idx]
            instr.append(
                f"    {code} LP[{point_idx}], V={speed}, Z={br}, "
                f"Tool[{p['utool']}], Wobj[{p['uframe']}];"
            )
        elif t == "wait":
            instr.append(f"    Delay {duration_sec(node)};")
        elif t in ("set_do", "set_ao"):
            instr.append(f"    // Set {io_port(node)}")
        else:
            instr.append(f"    // skip {t}")

    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    with open(out, "w", encoding="utf-8", buffering=1024 * 1024) as f:
        f.write(f"ProgramInfo {name}\n")
        f.write(f"// {comment}\n")
        for idx, p in points.items():
            f.write(
                f"LP[{idx}] = {p['x']:9.6f}, {p['y']:9.6f}, {p['z']:9.6f}, "
                f"{p['rx']:9.6f}, {p['ry']:9.6f}, {p['rz']:9.6f}; "
                f"0, 0, 0; 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0,0,0;\n"
            )
        f.write("Func run()\n")
        for line in instr:
            f.write(line)
            f.write("\n")
        f.write("EndFunc;\n")


ExportScript, run = export_entry(convert)
