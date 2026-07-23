# -*- coding: utf-8 -*-
"""CloudSim Canonical v1 读取与遍历（品牌导出共用）"""

from __future__ import annotations

import json
import math
import os
from typing import Any, Iterator

FORMAT_ID = "cloudsim.program_export"
SCHEMA_VERSION = 1


def load_json_file(path: str) -> dict[str, Any]:
    if not os.path.exists(path):
        raise FileNotFoundError(f"未找到文件: {path}")
    # Canonical 由 C++ 写 utf-8；优先单次解码，避免万级点四次试读
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except (UnicodeDecodeError, json.JSONDecodeError):
        pass
    last_err: Exception | None = None
    for enc in ("utf-8-sig", "gbk", "gb2312"):
        try:
            with open(path, "r", encoding=enc) as f:
                return json.load(f)
        except (UnicodeDecodeError, json.JSONDecodeError) as e:
            last_err = e
            continue
    raise ValueError(f"无法读取 JSON: {path} ({last_err})")


def load_canonical_v1(path: str) -> dict[str, Any]:
    root = load_json_file(path)
    if root.get("format") != FORMAT_ID:
        raise ValueError(f"format 须为 {FORMAT_ID}，实际: {root.get('format')}")
    if int(root.get("schemaVersion", 0)) != SCHEMA_VERSION:
        raise ValueError(f"不支持 schemaVersion={root.get('schemaVersion')}")
    return root


def _vec3(v: Any) -> tuple[float, float, float]:
    if isinstance(v, dict):
        return float(v.get("x", 0.0)), float(v.get("y", 0.0)), float(v.get("z", 0.0))
    if isinstance(v, (list, tuple)) and len(v) >= 3:
        return float(v[0]), float(v[1]), float(v[2])
    return 0.0, 0.0, 0.0


def pose_mm_deg(node: dict[str, Any]) -> dict[str, float]:
    """位姿：mm + 欧拉角度（Canonical 约定）"""
    poses = node.get("poses") or {}
    base = poses.get("baseTcp") or {}
    x, y, z = _vec3(base.get("positionMm"))
    rx, ry, rz = _vec3(base.get("eulerDeg"))
    return {"x": x, "y": y, "z": z, "rx": rx, "ry": ry, "rz": rz}


def pose_mm_rad(node: dict[str, Any]) -> dict[str, float]:
    p = pose_mm_deg(node)
    return {
        "x": p["x"],
        "y": p["y"],
        "z": p["z"],
        "rx": math.radians(p["rx"]),
        "ry": math.radians(p["ry"]),
        "rz": math.radians(p["rz"]),
    }


def motion_speed(node: dict[str, Any], default: float = 100.0) -> float:
    m = node.get("motion") or {}
    return float(m.get("speed", default))


def motion_accel(node: dict[str, Any], default: float = 500.0) -> float:
    m = node.get("motion") or {}
    return float(m.get("accel", default))


def motion_blend(node: dict[str, Any], default: float = 100.0) -> float:
    m = node.get("motion") or {}
    return float(m.get("blendRadius", default))


def tool_frame_name(node: dict[str, Any]) -> str:
    poses = node.get("poses") or {}
    fid = poses.get("toolFrameId") or ""
    if fid:
        return str(fid)
    return "Tool0"


def user_frame_name(node: dict[str, Any]) -> str:
    poses = node.get("poses") or {}
    fid = poses.get("userFrameId") or ""
    if fid:
        return str(fid)
    return "Wobj0"


def condition_text(node: dict[str, Any]) -> str:
    logic = node.get("logic") or {}
    cond = logic.get("condition")
    if cond is None:
        return ""
    if isinstance(cond, str):
        return cond
    if isinstance(cond, dict):
        t = str(cond.get("type", "")).lower()
        if t in ("always", ""):
            return "TRUE"
        if t == "never":
            return "FALSE"
        if "expr" in cond:
            return str(cond["expr"])
        if "text" in cond:
            return str(cond["text"])
        return json.dumps(cond, ensure_ascii=False)
    return str(cond)


def duration_sec(node: dict[str, Any]) -> float:
    logic = node.get("logic") or {}
    return float(logic.get("durationSec", 0.0))


def io_port(node: dict[str, Any]) -> str:
    logic = node.get("logic") or {}
    return str(logic.get("port", ""))


def program_name_from_doc(doc: dict[str, Any], fallback: str = "Main") -> str:
    prog = doc.get("program") or {}
    name = str(prog.get("name") or prog.get("id") or fallback)
    cleaned = "".join(c for c in name if c.isalnum() or c == "_")
    if not cleaned:
        cleaned = fallback
    if not cleaned[0].isalpha():
        cleaned = "M_" + cleaned
    return cleaned


def walk_nodes(records: list[dict[str, Any]]) -> Iterator[tuple[str, dict[str, Any]]]:
    """
    DFS 产出 (事件, 节点)。
    事件: node | if_enter | if_else | if_leave | while_enter | while_leave
    """
    for rec in records:
        t = str(rec.get("type", "")).lower()
        if t == "if":
            yield ("if_enter", rec)
            then_body = rec.get("then")
            if then_body:
                yield from walk_nodes(then_body)
            else_body = rec.get("else")
            if else_body:
                yield ("if_else", rec)
                yield from walk_nodes(else_body)
            yield ("if_leave", rec)
        elif t == "while":
            yield ("while_enter", rec)
            body = rec.get("body")
            if body:
                yield from walk_nodes(body)
            yield ("while_leave", rec)
        else:
            yield ("node", rec)


def safe_module_name(path: str, default: str = "Program") -> str:
    base = os.path.splitext(os.path.basename(path))[0]
    name = "".join(c for c in base if c.isalnum() or c == "_")
    if not name:
        name = default
    if not name[0].isalpha():
        name = "M_" + name
    return name


def ensure_output_ext(path: str, ext: str) -> str:
    root, cur = os.path.splitext(path)
    if cur.lower() != ext.lower():
        return root + ext
    return path


def export_entry(convert_fn):
    """包装 ExportScript / run 统一入口"""

    def ExportScript(OutPutPath: str, CanonicalPath: str, comment: str = "") -> str:
        try:
            convert_fn(OutPutPath, CanonicalPath, comment)
            return "true"
        except Exception as e:
            print(f"[ExportScript] 失败: {e}")
            return "false"

    def run(params: dict) -> dict:
        try:
            out = str(params.get("OutPutPath", ""))
            src = str(params.get("CanonicalPath", ""))
            comment = str(params.get("comment", ""))
            convert_fn(out, src, comment)
            return {"ok": True, "error": ""}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    return ExportScript, run
