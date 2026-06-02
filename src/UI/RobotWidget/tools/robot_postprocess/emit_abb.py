"""ABB RAPID stub emitter from canonical v1."""

from __future__ import annotations

from read_canonical_v1 import CanonicalProgramV1, iter_nested


def emit_rapid_stub(doc: CanonicalProgramV1) -> str:
    lines = ["MODULE CloudSimExport", "    ! Generated from cloudsim.program_export v1", "    PROC main()"]
    for rec in iter_nested(doc.instructions):
        t = rec.get("type", "")
        if t == "if":
            lines.append("        IF <condition> THEN")
            continue
        if t == "while":
            lines.append("        WHILE <condition> DO")
            continue
        if not rec.get("executable", False):
            continue
        if t in ("ptp", "line"):
            poses = rec.get("poses", {}).get("baseTcp", {})
            pos = poses.get("positionMm", [0, 0, 0])
            lines.append(
                f"        MoveJ [{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}], v100, fine, tool0;"
                if t == "ptp"
                else f"        MoveL [{pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f}], v200, fine, tool0;"
            )
        elif t == "wait":
            dur = rec.get("logic", {}).get("durationSec", 1.0)
            lines.append(f"        WaitTime {dur};")
    lines.extend(["    ENDPROC", "ENDMODULE"])
    return "\n".join(lines) + "\n"
