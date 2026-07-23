# -*- coding: utf-8 -*-
"""珞石 ROKAE：Canonical v1 → .mod"""

from __future__ import annotations

import math
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


def _euler_to_quat(rx, ry, rz):
    rx_rad, ry_rad, rz_rad = math.radians(rx), math.radians(ry), math.radians(rz)
    cx, sx = math.cos(rx_rad / 2), math.sin(rx_rad / 2)
    cy, sy = math.cos(ry_rad / 2), math.sin(ry_rad / 2)
    cz, sz = math.cos(rz_rad / 2), math.sin(rz_rad / 2)
    return (
        cx * cy * cz + sx * sy * sz,
        sx * cy * cz - cx * sy * sz,
        cx * sy * cz + sx * cy * sz,
        cx * cy * sz - sx * sy * cz,
    )


def convert(OutPutPath: str, CanonicalPath: str, comment: str = "") -> None:
    doc = load_canonical_v1(CanonicalPath)
    out = ensure_output_ext(OutPutPath, ".mod")
    # 珞石习惯带 _rokae 后缀时保留用户主名
    comment = comment or f"From Canonical {os.path.basename(CanonicalPath)} @ {datetime.now():%Y-%m-%d %H:%M:%S}"

    points: dict[int, dict] = {}
    instr: list[str] = []
    point_idx = 0

    for event, node in walk_nodes(doc.get("instructions") or []):
        t = str(node.get("type", "")).lower()
        if event == "if_enter":
            instr.append(f"        // IF {condition_text(node)}")
            continue
        if event == "if_else":
            instr.append("        // ELSE")
            continue
        if event == "if_leave":
            instr.append("        // ENDIF")
            continue
        if event == "while_enter":
            instr.append(f"        // WHILE {condition_text(node)}")
            continue
        if event == "while_leave":
            instr.append("        // ENDWHILE")
            continue
        if event != "node":
            continue

        if t in ("ptp", "line"):
            point_idx += 1
            pose = pose_mm_deg(node)
            points[point_idx] = pose
            speed = int(motion_speed(node, 100 if t == "ptp" else 500))
            br = int(motion_blend(node, 100))
            zone = "fine" if br <= 0 else f"z{br}"
            tool = (tool_frame_name(node) or "tool0").lower()
            wobj = (user_frame_name(node) or "wobj0").lower()
            move = "MoveJ" if t == "ptp" else "MoveL"
            instr.append(f"        {move}( p{point_idx}, v{speed}, {zone}, {tool}, {wobj});")
        elif t == "wait":
            instr.append(f"        WaitTime({duration_sec(node)});")
        elif t in ("set_do", "set_ao"):
            instr.append(f"        // Set {io_port(node)}")
        else:
            instr.append(f"        // skip {t}")

    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    with open(out, "w", encoding="utf-8", buffering=1024 * 1024) as f:
        f.write(f"\n// {comment}\nPROC main()\n")
        for idx, pose in points.items():
            q1, q2, q3, q4 = _euler_to_quat(pose["rx"], pose["ry"], pose["rz"])
            f.write(
                f"CONST robtarget p{idx} = p:{{{pose['x']:.6f}, {pose['y']:.6f}, {pose['z']:.6f}}},"
                f"{{{q1:.6f}, {q2:.6f}, {q3:.6f}, {q4:.6f}}}, 0}}"
                f"{{cfg 0,0,0,0,0,0,0,0}}"
                f"{{EJ 0.000000,0.000000,0.000000,0.000000,0.000000,0.000000}};\n"
            )
        for line in instr:
            f.write(line)
            f.write("\n")
        f.write("ENDPROC\n")


ExportScript, run = export_entry(convert)
