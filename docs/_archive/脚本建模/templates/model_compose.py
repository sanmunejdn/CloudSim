# -*- coding: utf-8 -*-
"""feature.compose 建模模板 — CloudSim 几何模式「Python」控制台。

覆盖 Compose 支持的全部 api（无 Mirror3D / TwoDirections → 见 model_history.py）。
"""

from __future__ import annotations

import json
import cloudsim_geom as g


def build_compose_plan():
    return {
        "version": 2,
        "domain": "feature.compose",
        "steps": [
            {
                "id": "body",
                "api": "extrudeSketchProfileToBrep",
                "args": {
                    "mode": "pad",
                    "profile": "rectangle",  # rectangle|polygon|circle 或 profile_xyz_mm
                    "length_mm": 80,
                    "width_mm": 50,
                    "extrude_mm": 20,
                    "end_condition": "blind",  # blind|through_all|mid_plane
                    "name": "Body",
                },
            },
            {
                "id": "hole",
                "api": "extrudeSketchProfileToBrep",
                "args": {
                    "mode": "pocket",
                    "profile": "circle",
                    "diameter_mm": 10,
                    "center_u_mm": 40,
                    "center_v_mm": 25,
                    "end_condition": "through_all",
                    "extrude_mm": 20,
                    "target": "$body",
                },
            },
            {
                "id": "fillet",
                "api": "filletEdgesToBrep",
                "args": {
                    "target": "$body",
                    "radius_mm": 2,
                    "edges": "longest",  # longest|top_boundary|all 或 edge_indices
                    "edge_count": 4,
                },
            },
            {
                "id": "chamfer",
                "api": "chamferEdgesToBrep",
                "args": {
                    "target": "$body",
                    "distance_mm": 1,
                    "edges": "top_boundary",
                },
            },
            {
                "id": "revolve_boss",
                "api": "revolveSketchProfileToBrep",
                "args": {
                    "mode": "boss",  # boss|cut（cut 需 target）
                    "profile": "rectangle",
                    "length_mm": 8,
                    "width_mm": 20,
                    "angle_deg": 360,
                    "axis_ox": 100,
                    "axis_oy": 0,
                    "axis_oz": 0,
                    "axis_dx": 0,
                    "axis_dy": 0,
                    "axis_dz": 1,
                    "name": "RevolveBoss",
                },
            },
            {
                "id": "linpat",
                "api": "linearPatternBodyToBrep",
                "args": {
                    "target": "$body",
                    "count": 3,
                    "dx_mm": 90,
                    "dy_mm": 0,
                    "dz_mm": 0,
                    # "source_feature_id": "Pad1",
                },
            },
            {
                "id": "circpat",
                "api": "circularPatternBodyToBrep",
                "args": {
                    "target": "$body",
                    "count": 4,
                    "angle_deg": 360,
                    "axis_ox": 40,
                    "axis_oy": 25,
                    "axis_oz": 0,
                    "axis_dx": 0,
                    "axis_dy": 0,
                    "axis_dz": 1,
                },
            },
            {
                "id": "sweep",
                "api": "sweepSketchProfileToBrep",
                "args": {
                    "mode": "boss",
                    "profile": "circle",
                    "diameter_mm": 6,
                    "path": "line_z",
                    "path_length_mm": 40,
                    "twist_deg": 0,
                    "name": "SweepBoss",
                },
            },
            {
                "id": "loft",
                "api": "loftSketchProfilesToBrep",
                "args": {
                    "mode": "boss",
                    "profile_a": "rectangle",
                    "length_a_mm": 20,
                    "width_a_mm": 20,
                    "profile_b": "circle",
                    "diameter_b_mm": 12,
                    "profile_b_z_mm": 30,
                    "name": "LoftBoss",
                },
            },
            {
                "id": "shell",
                "api": "shellFacesToBrep",
                "args": {
                    "target": "$body",
                    "thickness_mm": 1.5,
                    "face_indices": [0],  # 必填，按拓扑修改
                },
            },
            {
                "id": "draft",
                "api": "draftFacesToBrep",
                "args": {
                    "target": "$body",
                    "angle_deg": 2,
                    "face_indices": [1, 2],
                    "neutral_ox": 0,
                    "neutral_oy": 0,
                    "neutral_oz": 0,
                    "neutral_nx": 0,
                    "neutral_ny": 0,
                    "neutral_nz": 1,
                },
            },
        ],
    }


def main():
    plan = build_compose_plan()
    # 可按需删掉 shell/draft 步，避免 face_indices 不匹配导致失败
    plan["steps"] = [s for s in plan["steps"] if s["id"] not in ("shell", "draft")]
    summary = g.run_compose(json.dumps(plan, ensure_ascii=False))
    print("summary:", summary)
    print("bodies:", g.list_bodies())


if __name__ == "__main__":
    main()
