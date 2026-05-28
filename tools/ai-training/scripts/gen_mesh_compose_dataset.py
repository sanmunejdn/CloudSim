#!/usr/bin/env python3
"""生成 mesh.compose/dataset.jsonl（避免 shell 吃掉 $ 引用）。"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

MQ = {"segments": 32}


def plan(steps):
    return json.dumps(
        {"version": 2, "domain": "mesh.compose", "steps": steps},
        ensure_ascii=False,
        separators=(",", ":"),
    )


def _pose(x=0, y=0, z=0):
    return {"x": x, "y": y, "z": z}


def box_step(sid, l, w, h, name="Body", pose=None):
    args = {
        "primitive": "box",
        "dimensions_mm": {"length": l, "width": w, "height": h},
        "name": name,
        "mesh_quality": MQ,
    }
    if pose is not None:
        args["pose_mm"] = pose
    return {"id": sid, "api": "createPrimitiveMesh", "args": args}


def cyl_step(sid, r, h, name="Cylinder", pose=None):
    args = {
        "primitive": "cylinder",
        "dimensions_mm": {"radius": r, "height": h},
        "name": name,
        "mesh_quality": MQ,
    }
    if pose is not None:
        args["pose_mm"] = pose
    return {"id": sid, "api": "createPrimitiveMesh", "args": args}


def bool_step(sid, target, tool, op="difference", rname="Result"):
    return {
        "id": sid,
        "api": "booleanMesh",
        "args": {
            "op": op,
            "target": target,
            "tool": tool,
            "result_name": rname,
            "hide_operands": True,
        },
    }


def row(instruction, steps):
    return {"instruction": instruction, "input": "", "output": plan(steps)}


def hole_plan(l, w, h, r, hh, rname="BoxWithHole"):
    return [
        box_step("body", l, w, h),
        cyl_step("hole_tool", r, hh, "HoleTool"),
        bool_step("result", "$body", "$hole_tool", "difference", rname),
    ]


def union_boxes_offset(la, wa, ha, lb, wb, hb, ox, oy=0, oz=0):
    return [
        box_step("a", la, wa, ha, "A"),
        box_step("b", lb, wb, hb, "B", _pose(ox, oy, oz)),
        bool_step("u", "$a", "$b", "union", "UnionAB"),
    ]


def intersect_boxes_offset(la, wa, ha, lb, wb, hb, ox, oy=0, oz=0):
    return [
        box_step("a", la, wa, ha, "A"),
        box_step("b", lb, wb, hb, "B", _pose(ox, oy, oz)),
        bool_step("i", "$a", "$b", "intersection", "Intersect"),
    ]


def union_box_cylinder_boss(l, w, h, r, ch, cz):
    return [
        box_step("base", l, w, h, "Base"),
        cyl_step("boss", r, ch, "Boss", _pose(0, 0, cz)),
        bool_step("u", "$base", "$boss", "union", "UnionBoss"),
    ]


def union_cylinders_offset(r1, h1, r2, h2, ox):
    return [
        cyl_step("a", r1, h1, "CylA"),
        cyl_step("b", r2, h2, "CylB", _pose(ox, 0, 0)),
        bool_step("u", "$a", "$b", "union", "UnionCyl"),
    ]


def intersect_box_cylinder(l, w, h, r, ch):
    return [
        box_step("a", l, w, h, "Box"),
        cyl_step("b", r, ch, "Cyl"),
        bool_step("i", "$a", "$b", "intersection", "Intersect"),
    ]


def add_difference_samples(rows):
    rows.append(
        row(
            "生成长方体，长宽高为100,100,100，在顶部挖一个直径50的通孔",
            hole_plan(100, 100, 100, 25, 120),
        )
    )
    for ins in [
        "生成100立方体中心通孔直径50",
        "做一个长宽高100的盒子挖直径50的孔",
        "长方体100x100x100通孔D50",
        "create box 100 with through hole diameter 50",
        "100mm cube through hole 50mm",
        "生成长方体，长宽高100,100,200，在顶部挖直径50通孔",
        "生成长方体挖通孔",
        "创建盒子并打孔",
        "生成带孔的方块",
    ]:
        h = 200 if "200" in ins else 100
        hh = int(h * 1.2 + 20)
        rows.append(row(ins, hole_plan(100, 100, h, 25, hh)))

    for ins in ["长方体100挖盲孔直径40深30", "box 100 blind hole d40 depth 30"]:
        rows.append(
            row(
                ins,
                [
                    box_step("body", 100, 100, 100),
                    cyl_step("hole_tool", 20, 30),
                    bool_step("result", "$body", "$hole_tool"),
                ],
            )
        )

    for ins in ["大孔直径60的通孔", "大一点孔径60"]:
        rows.append(row(ins, hole_plan(100, 100, 100, 30, 130)))

    rows.append(
        row(
            "扁盒子上打两个小孔",
            [
                box_step("body", 120, 80, 60),
                cyl_step("h1", 15, 80),
                bool_step("t1", "$body", "$h1"),
                cyl_step("h2", 10, 80),
                bool_step("out", "$t1", "$h2"),
            ],
        )
    )

    # 反例：含并集字样但意图是挖孔 → difference
    for ins in [
        "两个盒子并集后挖直径50通孔",
        "union then drill through hole D50 in 100 cube",
    ]:
        rows.append(row(ins, hole_plan(100, 100, 100, 25, 120)))


def add_union_samples(rows):
    off = union_boxes_offset(80, 80, 80, 60, 60, 60, 40)
    for ins in [
        "两个长方体并集",
        "union two boxes",
        "80和60的两个盒子并在一起，小盒X方向偏40",
        "union box 80 and box 60 offset x 40",
        "合并两个长方体80与60，小盒沿X偏40mm",
        "把两个盒子拼在一起 80立方和60立方 X偏移40",
        "combine two boxes 80mm and 60mm offset x 40",
        "两个盒子合在一起",
        "长方体并集 大80 小60 错位",
    ]:
        rows.append(row(ins, off))

    boss = union_box_cylinder_boss(100, 100, 40, 30, 50, 45)
    for ins in [
        "长方体底座上加圆柱凸台 R30 H50",
        "box 100x100x40 union cylinder boss R30 H50 on top",
        "底座盒子并圆柱凸台",
        "附加圆柱凸台到底座",
    ]:
        rows.append(row(ins, boss))

    cyl_u = union_cylinders_offset(40, 80, 25, 60, 35)
    for ins in [
        "两个圆柱并集 R40 H80 与 R25 H60 X偏35",
        "union two cylinders offset x 35",
        "圆柱拼合",
    ]:
        rows.append(row(ins, cyl_u))

    # 同中心（基础课，少量）
    same = [
        box_step("a", 80, 80, 80, "A"),
        box_step("b", 60, 60, 60, "B"),
        bool_step("u", "$a", "$b", "union", "UnionAB"),
    ]
    rows.append(row("两个同中心盒子并集 80与60", same))


def add_intersection_samples(rows):
    off = intersect_boxes_offset(100, 100, 100, 80, 80, 80, 50)
    for ins in [
        "两盒子求交 100与80 X偏50",
        "intersection of two boxes 100 and 80 offset x 50",
        "100和80盒子求交，80盒X偏50",
        "保留两个盒子的重叠部分",
        "求两个长方体的交集",
        "intersection two boxes offset",
        "重叠部分 盒子100 80",
    ]:
        rows.append(row(ins, off))

    bc = intersect_box_cylinder(100, 100, 100, 40, 120)
    for ins in [
        "长方体与圆柱求交",
        "box 100 cube intersection cylinder R40",
        "盒子与圆柱交集",
    ]:
        rows.append(row(ins, bc))

    same = [
        box_step("a", 100, 100, 100, "A"),
        box_step("b", 80, 80, 80, "B"),
        bool_step("i", "$a", "$b", "intersection", "Intersect"),
    ]
    for ins in ["两盒子求交 同中心", "intersection two centered boxes 100 and 80"]:
        rows.append(row(ins, same))

    # 反例
    rows.append(row("两个盒子并集 80 60", union_boxes_offset(80, 80, 80, 60, 60, 60, 40)))
    rows.append(row("100盒挖通孔D50", hole_plan(100, 100, 100, 25, 120)))


def add_create_only(rows):
    for ins in ["只生成长方体100", "single box 100"]:
        rows.append(row(ins, [box_step("body", 100, 100, 100)]))


def count_ops(rows):
    from collections import Counter

    c = Counter()
    for r in rows:
        out = json.loads(r["output"])
        for step in out.get("steps", []):
            if step.get("api") == "booleanMesh":
                c[step["args"].get("op", "difference")] += 1
                break
        else:
            c["create_only"] += 1
    return c


def main():
    rows = []
    add_difference_samples(rows)
    add_union_samples(rows)
    add_intersection_samples(rows)
    add_create_only(rows)

    stats = count_ops(rows)
    print(f"samples: {len(rows)} ops: {dict(stats)}")

    path = ROOT / "domains" / "mesh.compose" / "dataset.jsonl"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    print(f"wrote {len(rows)} lines to {path}")


if __name__ == "__main__":
    main()
