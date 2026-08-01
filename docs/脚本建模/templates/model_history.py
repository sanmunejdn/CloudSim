# -*- coding: utf-8 -*-
"""History 建模模板 — 在 CloudSim 几何模式「Python」控制台中运行。

覆盖：草图实体（点/线/弧/圆/椭圆/样条）+ 约束枚举 + 常用实体特征字段。
用法：
  1. 进入几何建模工作区并打开文档
  2. Ribbon → Python → 粘贴本脚本 → Run
  或：按 JSON 路径读文件后 g.import_history(...)
"""

from __future__ import annotations

import json
import cloudsim_geom as g

# ---------------------------------------------------------------------------
# 约束 kind（与 SketchGeom SkConstraintKind 一致）
# ---------------------------------------------------------------------------
COINCIDENT = 0
HORIZONTAL = 1
VERTICAL = 2
EQUAL_LENGTH = 3
DISTANCE = 4
PARALLEL = 5
PERPENDICULAR = 6
RADIUS = 7
ANGLE = 8
ARC_RADIUS = 9
TANGENT = 10
SYMMETRIC = 11
MIDPOINT = 12
MAJOR_RADIUS = 13
MINOR_RADIUS = 14

XY = {
    "origin": [0, 0, 0],
    "axisX": [1, 0, 0],
    "axisY": [0, 1, 0],
    "normal": [0, 0, 1],
    "isPlanar": True,
}


def closed_rect_xyz(x0, y0, z, w, h):
    """世界坐标闭合矩形折线（末点回到起点）。"""
    return [
        x0, y0, z,
        x0 + w, y0, z,
        x0 + w, y0 + h, z,
        x0, y0 + h, z,
        x0, y0, z,
    ]


def build_sketch_document():
    """草图命令/实体全覆盖示例（嵌入 Sketch.sketchDocument）。"""
    return {
        "seq": 30,
        "points": [
            {"id": 1, "u": 0, "v": 0, "fixed": True},
            {"id": 2, "u": 60, "v": 0, "fixed": False},
            {"id": 3, "u": 60, "v": 40, "fixed": False},
            {"id": 4, "u": 0, "v": 40, "fixed": False},
            {"id": 5, "u": 30, "v": 20, "fixed": False},
            {"id": 6, "u": 20, "v": 10, "fixed": False},
            {"id": 7, "u": 40, "v": 10, "fixed": False},
            {"id": 8, "u": 30, "v": 30, "fixed": False},
            {"id": 9, "u": 10, "v": 20, "fixed": False},
            {"id": 10, "u": 15, "v": 25, "fixed": False},
            {"id": 11, "u": 25, "v": 25, "fixed": False},
        ],
        "lines": [
            {"id": 100, "p1": 1, "p2": 2, "construction": False},  # 直线
            {"id": 101, "p1": 2, "p2": 3, "construction": False},
            {"id": 102, "p1": 3, "p2": 4, "construction": False},
            {"id": 103, "p1": 4, "p2": 1, "construction": False},
            {"id": 104, "p1": 1, "p2": 3, "construction": True},  # 构造线
        ],
        "arcs": [
            {"id": 200, "pStart": 6, "pMid": 8, "pEnd": 7, "construction": False},
        ],
        "circles": [
            {"id": 300, "center": 5, "radius": 5, "construction": False},
        ],
        "ellipses": [
            {
                "id": 400,
                "center": 9,
                "majorR": 8,
                "minorR": 4,
                "angleRad": 0.3,
                "construction": False,
            },
        ],
        "splines": [
            {
                "id": 500,
                "points": [10, 11, 8],  # 过点
                "controlPoints": [],
                "mode": 0,  # 0=ThroughPoints, 1=ControlPoints
                "construction": True,
            },
        ],
        "constraints": [
            {"kind": HORIZONTAL, "a": 100, "b": -1, "value": 0},
            {"kind": VERTICAL, "a": 101, "b": -1, "value": 0},
            {"kind": DISTANCE, "a": 100, "b": -1, "value": 60},
            {"kind": DISTANCE, "a": 101, "b": -1, "value": 40},
            {"kind": PARALLEL, "a": 100, "b": 102, "value": 0},
            {"kind": PERPENDICULAR, "a": 100, "b": 101, "value": 0},
            {"kind": EQUAL_LENGTH, "a": 100, "b": 102, "value": 0},
            {"kind": RADIUS, "a": 300, "b": -1, "value": 5},
            {"kind": ARC_RADIUS, "a": 200, "b": -1, "value": 0},
            {"kind": MAJOR_RADIUS, "a": 400, "b": -1, "value": 8},
            {"kind": MINOR_RADIUS, "a": 400, "b": -1, "value": 4},
            {"kind": TANGENT, "a": 200, "b": 100, "value": 0},
            {"kind": MIDPOINT, "a": 5, "b": 100, "value": 0},
            {"kind": SYMMETRIC, "a": 6, "b": 7, "value": 0, "c": 104},
            {"kind": ANGLE, "a": 100, "b": 101, "value": 90},
            {"kind": COINCIDENT, "a": 1, "b": 1, "value": 0},
        ],
    }


def build_history():
    """Parametric history：Sketch + Pad(+可选其它特征字段示范)。"""
    return {
        "seq": 4,
        "features": [
            {
                "id": "Sketch1",
                "name": "SketchFull",
                "kind": "Sketch",
                "plane": XY,
                "profile": closed_rect_xyz(0, 0, 0, 60, 40),
                "sketchDocument": build_sketch_document(),
                "suppressed": False,
                "visible": True,
            },
            {
                "id": "Pad1",
                "name": "Pad1",
                "kind": "Pad",
                "lengthMm": 12,
                "startOffsetMm": 0,
                "length2Mm": 0,
                "endCondition": "Blind",  # Blind|MidPlane|TwoDirections|ThroughAll|...
                "draftAngleDeg": 0,
                "reversed": False,
                "sketchRefId": "Sketch1",
            },
            # --- 以下为字段示范，默认 suppressed，按需打开并改索引 ---
            {
                "id": "Fillet1",
                "kind": "Fillet",
                "edgeIndices": [0, 1, 2, 3],
                "radiusMm": 1.5,
                "suppressed": True,
            },
            {
                "id": "Chamfer1",
                "kind": "Chamfer",
                "edgeIndices": [4],
                "chamferDistMm": 1.0,
                "suppressed": True,
            },
            {
                "id": "LinPat1",
                "kind": "LinearPattern",
                "patternCount": 2,
                "patternD": [70, 0, 0],
                "patternSourceFeatureId": "Pad1",
                "suppressed": True,
            },
            {
                "id": "CircPat1",
                "kind": "CircularPattern",
                "patternCount": 4,
                "patternAngleDeg": 360,
                "axisO": [30, 20, 0],
                "axisD": [0, 0, 1],
                "patternSourceFeatureId": "Pad1",
                "suppressed": True,
            },
            {
                "id": "Mirror1",
                "kind": "Mirror3D",
                "mirrorPlane": {
                    "origin": [0, 0, 0],
                    "axisX": [0, 1, 0],
                    "axisY": [0, 0, 1],
                    "normal": [1, 0, 0],
                    "isPlanar": True,
                },
                "mirrorKeepOriginal": True,
                "suppressed": True,
            },
            {
                "id": "Shell1",
                "kind": "Shell",
                "faceIndices": [0],
                "shellThicknessMm": 1.5,
                "suppressed": True,
            },
            {
                "id": "Draft1",
                "kind": "Draft",
                "faceIndices": [1, 2],
                "draftAngleDeg": 3,
                "suppressed": True,
            },
            {
                "id": "PadTwoDir",
                "kind": "Pad",
                "lengthMm": 10,
                "length2Mm": 5,
                "endCondition": "TwoDirections",
                "sketchRefId": "Sketch1",
                "suppressed": True,
            },
        ],
    }


def main():
    hist = build_history()
    payload = json.dumps(hist, ensure_ascii=False)
    print("bodies before:", g.list_bodies())
    g.import_history(payload)  # body_id=None → 活动 Body 或新建
    print("bodies after:", g.list_bodies())
    print("exported bytes:", len(g.export_history()))


if __name__ == "__main__":
    main()
