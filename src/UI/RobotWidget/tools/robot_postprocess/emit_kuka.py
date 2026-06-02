"""KUKA KRL/SRC stub emitter from canonical v1."""

from __future__ import annotations

from read_canonical_v1 import CanonicalProgramV1, iter_nested


def emit_src_stub(doc: CanonicalProgramV1) -> str:
    lines = ["&ACCESS RVP", "&REL 1", "DEF CloudSimExport()", "    ; Generated from cloudsim.program_export v1"]
    for rec in iter_nested(doc.instructions):
        t = rec.get("type", "")
        if t == "if":
            lines.append("    IF <condition> THEN")
            continue
        if t == "while":
            lines.append("    WHILE <condition>")
            continue
        if not rec.get("executable", False):
            continue
        if t in ("ptp", "line"):
            poses = rec.get("poses", {}).get("baseTcp", {})
            pos = poses.get("positionMm", [0, 0, 0])
            motion = "PTP" if t == "ptp" else "LIN"
            lines.append(f"    {motion} {{X {pos[0]:.3f}, Y {pos[1]:.3f}, Z {pos[2]:.3f}}} C_DIS")
        elif t == "wait":
            dur = rec.get("logic", {}).get("durationSec", 1.0)
            lines.append(f"    WAIT SEC {dur}")
    lines.append("END")
    return "\n".join(lines) + "\n"
