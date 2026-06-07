#!/usr/bin/env python3
"""生成 resource/trajectory/ops 下程序级块 JSON 与 CommonScope。

原子块（Resample/Offset/Weave 等）由 gen_atomic_ops.py 生成，勿在此写 resource/trajectory/raw/。
工艺预设见 ProcessFlowPresets.json（手工或单独脚本维护）。
"""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "src" / "Robot" / "RobotScene" / "resource" / "trajectory"

OPS = {
    "Approach": {
        "defaults": {"scope": {"kind": 1}, "params": {
            "approach.distanceMm": 20.0, "approach.directionMode": 1,
            "approach.insertMode": 0, "approach.segmentSelectMode": 0,
            "approach.segmentFrom": 1, "approach.segmentTo": 1,
            "approach.overrideSpeedEnabled": False, "approach.speedMmPerSec": 100.0,
        }},
        "fields": [
            {"key": "approach.distanceMm", "type": "Double", "labelZh": "进刀距离", "unit": "mm", "min": 0, "max": 10000, "default": 20.0, "group": "approach", "order": 0},
            {"key": "approach.directionMode", "type": "Enum", "labelZh": "方向模式", "defaultInt": 1, "group": "approach", "order": 1},
            {"key": "approach.speedMmPerSec", "type": "Double", "labelZh": "速度", "unit": "mm/s", "default": 100.0, "group": "approach", "order": 8},
        ],
    },
    "Retract": {
        "defaults": {"scope": {"kind": 1}, "params": {
            "retract.distanceMm": 20.0, "retract.directionMode": 1,
            "retract.insertMode": 0, "retract.segmentSelectMode": 0,
            "retract.overrideSpeedEnabled": False, "retract.speedMmPerSec": 100.0,
        }},
        "fields": [
            {"key": "retract.distanceMm", "type": "Double", "labelZh": "退刀距离", "unit": "mm", "default": 20.0, "group": "retract", "order": 0},
        ],
    },
    "Translate": {
        "defaults": {"scope": {"kind": 1}, "params": {}},
        "fields": [
            {"key": "translate.dxMm", "type": "Double", "labelZh": "ΔX(起点)", "unit": "mm", "default": 0.0, "order": 1},
            {"key": "translate.dyMm", "type": "Double", "labelZh": "ΔY(起点)", "unit": "mm", "default": 0.0, "order": 2},
            {"key": "translate.dzMm", "type": "Double", "labelZh": "ΔZ(起点)", "unit": "mm", "default": 0.0, "order": 3},
        ],
    },
    "Rotate": {
        "defaults": {"scope": {"kind": 1}, "params": {"rotate.axisZ": 1.0, "rotate.angleDeg": 0.0}},
        "fields": [
            {"key": "rotate.angleDeg", "type": "Double", "labelZh": "角度", "unit": "deg", "default": 0.0, "order": 4},
        ],
    },
    "Mirror": {"defaults": {"scope": {"kind": 1}, "params": {"mirror.axis": 0}}, "fields": [
        {"key": "mirror.axis", "type": "Int", "labelZh": "镜像轴", "defaultInt": 0, "order": 0},
    ]},
    "Delete": {"defaults": {"scope": {"kind": 1}, "params": {}}, "fields": []},
    "Duplicate": {"defaults": {"scope": {"kind": 1}, "params": {"structural.duplicateCount": 1}}, "fields": [
        {"key": "structural.duplicateCount", "type": "Int", "labelZh": "复制份数", "defaultInt": 1, "order": 0},
    ]},
    "Reorder": {"defaults": {"scope": {"kind": 1}, "params": {}}, "fields": []},
}


def write_op(name, spec, folder):
    folder.mkdir(parents=True, exist_ok=True)
    doc = {"version": 1, "kind": name, "schema": {"fields": spec["fields"]}, "defaults": spec["defaults"]}
    (folder / f"{name}.json").write_text(json.dumps(doc, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main():
    ops_dir = ROOT / "ops"
    for name, spec in OPS.items():
        write_op(name, spec, ops_dir)

    common = {
        "version": 1,
        "schema": {"fields": [
            {"key": "scope.kind", "type": "Enum", "labelZh": "作用域", "defaultInt": 1, "group": "scope", "order": 0},
            {"key": "scope.groupId", "type": "Enum", "labelZh": "分组", "group": "scope", "order": 1, "visibleWhenScopeKind": "Group"},
            {"key": "scope.pointFrom", "type": "Int", "labelZh": "P 起", "defaultInt": 1, "group": "scope", "order": 2, "visibleWhenScopeKind": "PointIndexRange"},
            {"key": "scope.pointTo", "type": "Int", "labelZh": "P 止", "defaultInt": 1, "group": "scope", "order": 3, "visibleWhenScopeKind": "PointIndexRange"},
        ]},
    }
    (ROOT / "CommonScope.json").write_text(json.dumps(common, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("json files written (ops/ + CommonScope only)")


if __name__ == "__main__":
    main()
