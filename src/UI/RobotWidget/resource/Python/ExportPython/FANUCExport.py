# -*- coding: utf-8 -*-
"""FANUC LS：Canonical v1 → .LS"""

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

CFX_MAP = {"0": "N U T, 0, 0, 0", "1": "F U T, 0, 0, 0", "2": "N D T, 0, 0, 0", "3": "F D T, 0, 0, 0"}
_DIGIT_RE = re.compile(r"\d+")


def _num(s: str) -> int:
    m = _DIGIT_RE.search(s or "")
    return int(m.group()) if m else 0


def convert(OutPutPath: str, CanonicalPath: str, comment: str = "") -> None:
    doc = load_canonical_v1(CanonicalPath)
    prog_name = safe_module_name(OutPutPath, program_name_from_doc(doc))
    out = ensure_output_ext(OutPutPath, ".LS")
    comment = comment or f"From Canonical {os.path.basename(CanonicalPath)} @ {datetime.now():%Y-%m-%d %H:%M:%S}"

    # 万级点：MN/POS 分缓冲，避免一次性 join 巨型字符串
    mn_parts: list[str] = []
    pos_parts: list[str] = []
    line_num = 1
    point_idx = 0
    cfg0 = CFX_MAP["0"]

    instructions = doc.get("instructions") or []
    for event, node in walk_nodes(instructions):
        t = str(node.get("type", "")).lower()
        if event == "if_enter":
            mn_parts.append(f"{line_num:4d}:  IF {condition_text(node) or '1'} THEN ;\n")
            line_num += 1
            continue
        if event == "if_else":
            mn_parts.append(f"{line_num:4d}:  ELSE ;\n")
            line_num += 1
            continue
        if event == "if_leave":
            mn_parts.append(f"{line_num:4d}:  ENDIF ;\n")
            line_num += 1
            continue
        if event == "while_enter":
            mn_parts.append(f"{line_num:4d}:  WHILE {condition_text(node) or '1'} ;\n")
            line_num += 1
            continue
        if event == "while_leave":
            mn_parts.append(f"{line_num:4d}:  ENDWHILE ;\n")
            line_num += 1
            continue
        if event != "node":
            continue

        if t in ("ptp", "line"):
            point_idx += 1
            pose = pose_mm_deg(node)
            uf = _num(user_frame_name(node))
            ut = _num(tool_frame_name(node))
            speed = int(motion_speed(node, 100 if t == "ptp" else 500))
            br = int(motion_blend(node, 100))
            if t == "ptp":
                mn_parts.append(f"{line_num:4d}:  J P[{point_idx}] {speed}% CNT{br} ;\n")
            else:
                mn_parts.append(f"{line_num:4d}:  L P[{point_idx}] {speed}mm/sec CNT{br} ;\n")
            line_num += 1
            pos_parts.append(
                f"P[{point_idx}]{{\n"
                f"   GP1:\n"
                f"\tUF : {uf}, UT : {ut},\n"
                f"\tJ1=     0.000 deg,\n"
                f"\tCONFIG : '{cfg0}',\n"
                f"\tX =   {pose['x']:8.3f} mm, Y =   {pose['y']:8.3f} mm, Z =   {pose['z']:8.3f} mm,\n"
                f"\tW =   {pose['rx']:8.3f} deg, P =   {pose['ry']:8.3f} deg, R =   {pose['rz']:8.3f} deg\n"
                f"}};\n"
            )
        elif t == "wait":
            mn_parts.append(f"{line_num:4d}:  WAIT   {duration_sec(node):.3f} ;\n")
            line_num += 1
        elif t == "set_do":
            port = io_port(node)
            val = 1 if (node.get("logic") or {}).get("digitalValue", True) else 0
            mn_parts.append(f"{line_num:4d}:  DO[{port}]={val} ;\n")
            line_num += 1
        else:
            mn_parts.append(f"{line_num:4d}:  ! skip {t} ;\n")
            line_num += 1

    now = datetime.now()
    date_str = now.strftime("%Y-%m-%d  TIME %H:%M:%S")
    attr_comment = comment[:40].replace('"', "'")

    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    with open(out, "w", encoding="utf-8", buffering=1024 * 1024) as f:
        f.write(f"/PROG  {prog_name}\n")
        f.write("/ATTR\n")
        f.write("OWNER\t\t= MNEDITOR;\n")
        f.write(f'COMMENT\t\t= "{attr_comment}";\n')
        f.write("PROG_SIZE\t= 0;\n")
        f.write(f"CREATE\t\t= DATE {date_str};\n")
        f.write(f"MODIFIED\t= DATE {date_str};\n")
        f.write(f"FILE_NAME\t= {prog_name};\n")
        f.write("VERSION\t\t= 0;\n")
        f.write("LINE_COUNT\t= 0;\n")
        f.write("MEMORY_SIZE\t= 0;\n")
        f.write("PROTECT\t\t= READ_WRITE;\n")
        f.write("TCD:  STACK_SIZE\t= 0,\n")
        f.write("      TASK_PRIORITY\t= 50,\n")
        f.write("      TIME_SLICE\t= 0,\n")
        f.write("      BUSY_LAMP_OFF\t= 0,\n")
        f.write("      ABORT_REQUEST\t= 0,\n")
        f.write("      PAUSE_REQUEST\t= 0;\n")
        f.write("DEFAULT_GROUP\t= 1,*,*,*,*;\n")
        f.write("CONTROL_CODE\t= 00000000 00000000;\n")
        f.write("/MN\n")
        f.writelines(mn_parts)
        f.write("/POS\n")
        f.writelines(pos_parts)
        f.write("/END\n")


ExportScript, run = export_entry(convert)
