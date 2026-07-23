# -*- coding: utf-8 -*-
"""ABB RAPID：Canonical v1 → .MOD"""

from __future__ import annotations

import math
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


def _extract_number(s: str) -> int:
    m = re.search(r"\d+", s or "")
    return int(m.group()) if m else 0


def _euler_to_quat(rx, ry, rz):
    rx_rad, ry_rad, rz_rad = math.radians(rx), math.radians(ry), math.radians(rz)
    cx, sx = math.cos(rx_rad / 2), math.sin(rx_rad / 2)
    cy, sy = math.cos(ry_rad / 2), math.sin(ry_rad / 2)
    cz, sz = math.cos(rz_rad / 2), math.sin(rz_rad / 2)
    q1 = cx * cy * cz + sx * sy * sz
    q2 = sx * cy * cz - cx * sy * sz
    q3 = cx * sy * cz + sx * cy * sz
    q4 = cx * cy * sz - sx * sy * cz
    return q1, q2, q3, q4


def convert(OutPutPath: str, CanonicalPath: str, comment: str = "") -> None:
    doc = load_canonical_v1(CanonicalPath)
    module_name = safe_module_name(OutPutPath, program_name_from_doc(doc))
    out = ensure_output_ext(OutPutPath, ".MOD")
    comment = comment or f"From Canonical {os.path.basename(CanonicalPath)} @ {datetime.now():%Y-%m-%d %H:%M:%S}"

    points: dict[int, dict] = {}
    instr_lines: list[str] = []
    point_idx = 0
    active_wobj, active_tool = "Wobj0", "Tool0"

    for event, node in walk_nodes(doc.get("instructions") or []):
        t = str(node.get("type", "")).lower()
        if event == "if_enter":
            instr_lines.append(f"        IF {condition_text(node) or 'TRUE'} THEN")
            continue
        if event == "if_else":
            instr_lines.append("        ELSE")
            continue
        if event == "if_leave":
            instr_lines.append("        ENDIF")
            continue
        if event == "while_enter":
            instr_lines.append(f"        WHILE {condition_text(node) or 'TRUE'} DO")
            continue
        if event == "while_leave":
            instr_lines.append("        ENDWHILE")
            continue
        if event != "node":
            continue

        if t in ("ptp", "line"):
            point_idx += 1
            pose = pose_mm_deg(node)
            wobj = user_frame_name(node) or active_wobj
            tool = tool_frame_name(node) or active_tool
            active_wobj, active_tool = wobj, tool
            speed = int(motion_speed(node, 100 if t == "ptp" else 500))
            br = int(motion_blend(node, 100))
            zone = "fine" if br <= 0 else f"z{br}"
            move = "MoveJ" if t == "ptp" else "MoveL"
            points[point_idx] = pose
            instr_lines.append(
                f"        {move} p{point_idx}, v{speed}, {zone}, {tool}\\WObj:={wobj};"
            )
        elif t == "wait":
            sec = duration_sec(node)
            instr_lines.append(f"        WaitTime {sec};")
        elif t == "set_do":
            port = io_port(node)
            val = (node.get("logic") or {}).get("digitalValue", True)
            instr_lines.append(f"        SetDO {port}, {'1' if val else '0'};")
        elif t == "set_ao":
            port = io_port(node)
            val = (node.get("logic") or {}).get("analogValue", 0)
            instr_lines.append(f"        SetAO {port}, {val};")
        else:
            name = node.get("name") or t
            instr_lines.append(f"        ! skip {name}")

    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    with open(out, "w", encoding="utf-8", buffering=1024 * 1024) as f:
        f.write(f"MODULE {module_name}\n")
        f.write(f"    ! {comment}\n\n")
        f.write("    PROC main()\n")
        for idx, pose in points.items():
            q1, q2, q3, q4 = _euler_to_quat(pose["rx"], pose["ry"], pose["rz"])
            f.write(
                f"    CONST robtarget p{idx}:="
                f"[[{pose['x']:.6f},{pose['y']:.6f},{pose['z']:.6f}],"
                f"[{q1:.6f},{q2:.6f},{q3:.6f},{q4:.6f}],"
                f"[0,0,0,0],"
                f"[9E9,9E9,9E9,9E9,9E9,9E9]];\n"
            )
        for line in instr_lines:
            f.write(line)
            f.write("\n")
        f.write("    ENDPROC\nENDMODULE\n")


ExportScript, run = export_entry(convert)

if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    # 本地自测需自行准备 Canonical JSON
    print("ABBExport ready")
