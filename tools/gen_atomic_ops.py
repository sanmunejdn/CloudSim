#!/usr/bin/env python3
"""生成 8 个原子轨迹算法块（Resample/Offset/Weave…）四件套与 JSON。"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILTINS = ROOT / "src" / "Robot" / "TrajectoryAlgorithmBuiltins" / "ops"
RES = ROOT / "src" / "Robot" / "RobotScene" / "resource" / "trajectory" / "ops"

ATOMIC_OPS = [
    {
        "name": "Resample",
        "kind": "Resample",
        "zh": "重采样",
        "prefix": "resample.",
        "fields": [("resample.stepMm", "Step", "点距", "mm", 5.0)],
        "struct_reads": [
            ('resample.stepMm', 'resample.stepMm', 'Double'),
        ],
        "math_fn": "resampleUnifiedTrajectory(traj, op.resample.stepMm)",
        "json_params": {"resample.stepMm": 5.0},
    },
    {
        "name": "OffsetAlongNormal",
        "kind": "OffsetAlongNormal",
        "zh": "法向偏移",
        "prefix": "offset.",
        "fields": [("offset.offsetMm", "Offset", "法向偏移", "mm", 0.0)],
        "struct_reads": [
            ('offset.offsetMm', 'pathOffset.offsetMm', 'Double'),
        ],
        "math_fn": "offsetAlongNormalUnified(traj, op.pathOffset.offsetMm)",
        "json_params": {"offset.offsetMm": 0.0},
    },
    {
        "name": "OffsetLateral",
        "kind": "OffsetLateral",
        "zh": "横向偏移",
        "prefix": "offset.",
        "fields": [("offset.lateralMm", "Lateral", "横向偏移", "mm", 0.0)],
        "struct_reads": [
            ('offset.lateralMm', 'pathOffset.lateralMm', 'Double'),
        ],
        "math_fn": "offsetLateralUnified(traj, op.pathOffset.lateralMm)",
        "json_params": {"offset.lateralMm": 0.0},
    },
    {
        "name": "SmoothPose",
        "kind": "SmoothPose",
        "zh": "姿态平滑",
        "prefix": None,
        "fields": [],
        "struct_reads": [],
        "math_fn": "smoothPoseUnified(traj)",
        "json_params": {},
    },
    {
        "name": "AssignBlend",
        "kind": "AssignBlend",
        "zh": "过渡半径",
        "prefix": "assign.",
        "fields": [("assign.blendRadiusMm", "Blend", "过渡半径", "mm", 2.0)],
        "struct_reads": [
            ('assign.blendRadiusMm', 'assignMotion.blendRadiusMm', 'Double'),
        ],
        "math_fn": "assignBlendUnified(traj, op.assignMotion.blendRadiusMm)",
        "json_params": {"assign.blendRadiusMm": 2.0},
    },
    {
        "name": "AssignSpeedZone",
        "kind": "AssignSpeedZone",
        "zh": "速度区",
        "prefix": "assign.",
        "fields": [("assign.speedMmPerSec", "Speed", "速度", "mm/s", 100.0)],
        "struct_reads": [
            ('assign.speedMmPerSec', 'assignMotion.speedMmPerSec', 'Double'),
        ],
        "math_fn": "assignSpeedUnified(traj, op.assignMotion.speedMmPerSec)",
        "json_params": {"assign.speedMmPerSec": 100.0},
    },
    {
        "name": "Weave",
        "kind": "Weave",
        "zh": "摆动",
        "prefix": "weave.",
        "fields": [
            ("weave.amplitudeMm", "Amplitude", "振幅", "mm", 2.0),
            ("weave.periodMm", "Period", "周期", "mm", 10.0),
        ],
        "struct_reads": [
            ('weave.amplitudeMm', 'weave.amplitudeMm', 'Double'),
            ('weave.periodMm', 'weave.periodMm', 'Double'),
        ],
        "math_fn": "weaveUnified(traj, op.weave.amplitudeMm, op.weave.periodMm)",
        "json_params": {"weave.amplitudeMm": 2.0, "weave.periodMm": 10.0},
    },
    {
        "name": "ReachabilityFilter",
        "kind": "ReachabilityFilter",
        "zh": "可达性过滤",
        "prefix": None,
        "fields": [],
        "struct_reads": [],
        "math_fn": "reachabilityFilterUnified(traj)",
        "json_params": {},
    },
    {
        "name": "ExternalAxisSearch",
        "kind": "ExternalAxisSearch",
        "zh": "外部轴搜索",
        "prefix": None,
        "fields": [],
        "struct_reads": [],
        "math_fn": "externalAxisSearchUnified(traj)",
        "json_params": {},
    },
]


def write_op_h(folder: Path, name: str):
    (folder / f"{name}Op.h").write_text(
        f"""#pragma once

#include "ITrajectoryOp.h"

namespace trajectory_algo
{{

class {name}Op final : public ITrajectoryOp
{{
public:
\tRobotInstruction::TrajectoryOpKind kind() const override;
\tconst char* displayName(bool chinese) const override;
\tTrajectoryOpCapability capabilities() const override;
\tRobotInstruction::TrajectoryOpDescriptor makeDefaultDescriptor(
\t\tconst RobotInstruction::OpScope& defaultScope) const override;
\tstd::vector<TrajectoryOpParamField> paramFields() const override;
\tbool validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const override;
\tstd::string formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, bool chinese) const override;
\tbool processPath(
\t\tconst RobotInstruction::TrajectoryOpDescriptor& op,
\t\tRobotInstruction::UnifiedTrajectory& traj,
\t\tstd::string* errMsg) const override;
}};

}} // namespace trajectory_algo
""",
        encoding="utf-8",
    )


def write_op_cpp(folder: Path, spec: dict):
    name = spec["name"]
    zh = spec["zh"]
    math_fn = spec["math_fn"]
    param_fields = []
    for i, (key, label_en, label_zh, unit, default) in enumerate(spec["fields"]):
        param_fields.append(
            f'\t\tdoubleParamField("{key}", "{label_en}", "{label_zh}", "{unit}", -1e5, 1e5, 0.01, {default}, {i}),'
        )
    param_fields_str = "\n".join(param_fields) if param_fields else ""
    if param_fields_str:
        param_fields_block = f"return {{\n{param_fields_str}\n\t}};"
    else:
        param_fields_block = "return {};"

    (folder / f"{name}Op.cpp").write_text(
        f"""#include "{name}Op.h"

#include "TrajectoryOpFormat.h"
#include "UnifiedTrajectoryPathMath.h"

namespace trajectory_algo
{{

RobotInstruction::TrajectoryOpKind {name}Op::kind() const
{{
\treturn RobotInstruction::TrajectoryOpKind::{spec['kind']};
}}

const char* {name}Op::displayName(const bool chinese) const
{{
\treturn chinese ? "{zh}" : "{spec['kind']}";
}}

TrajectoryOpCapability {name}Op::capabilities() const
{{
\treturn TrajectoryOpCapability::None;
}}

RobotInstruction::TrajectoryOpDescriptor {name}Op::makeDefaultDescriptor(
\tconst RobotInstruction::OpScope& defaultScope) const
{{
\tRobotInstruction::TrajectoryOpDescriptor op{{}};
\top.kind = RobotInstruction::TrajectoryOpKind::{spec['kind']};
\top.scope = defaultScope;
\treturn op;
}}

std::vector<TrajectoryOpParamField> {name}Op::paramFields() const
{{
\t{param_fields_block}
}}

bool {name}Op::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{{
\t(void)op;
\t(void)errMsg;
\treturn true;
}}

std::string {name}Op::formatSummary(
\tconst RobotInstruction::TrajectoryOpDescriptor& op,
\tconst bool chinese) const
{{
\t(void)op;
\treturn displayName(chinese);
}}

bool {name}Op::processPath(
\tconst RobotInstruction::TrajectoryOpDescriptor& op,
\tRobotInstruction::UnifiedTrajectory& traj,
\tstd::string* errMsg) const
{{
\t(void)op;
\t(void)errMsg;
\tif (traj.points.empty())
\t{{
\t\treturn false;
\t}}
\t{math_fn};
\treturn true;
}}

}} // namespace trajectory_algo
""",
        encoding="utf-8",
    )


def write_config(folder: Path, name: str, kind: str):
    (folder / f"{name}OpConfig.h").write_text(
        f"""#pragma once

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{{

std::unique_ptr<IOpParamConfig> make{name}OpConfig();

}} // namespace trajectory_algo
""",
        encoding="utf-8",
    )
    (folder / f"{name}OpConfig.cpp").write_text(
        f"""#include "{name}OpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{{

std::unique_ptr<IOpParamConfig> make{name}OpConfig()
{{
\treturn makeTrajectoryOpConfig(
\t\tRobotInstruction::TrajectoryOpKind::{kind},
\t\t"ops/{name}.json");
}}

}} // namespace trajectory_algo
""",
        encoding="utf-8",
    )


def write_param_access(folder: Path, spec: dict):
    name = spec["name"]
    prefix = spec["prefix"]
    reads = spec["struct_reads"]

    handles = (
        f'return key.rfind("{prefix}", 0) == 0;'
        if prefix
        else "return false;"
    )

    read_blocks = []
    write_blocks = []
    for key, member, vtype in reads:
        if vtype == "Double":
            read_blocks.append(
                f"""\tif (field.key == "{key}")
\t{{
\t\tout.kind = TrajectoryParamValue::Kind::Double;
\t\tout.asDouble = op.{member};
\t\treturn true;
\t}}"""
            )
            write_blocks.append(
                f"""\tif (field.key == "{key}")
\t{{
\t\top.{member} = in.asDouble;
\t\treturn true;
\t}}"""
            )

    (folder / f"{name}OpParamAccess.h").write_text(
        f"""#pragma once

#include "IOpParamAccess.h"

#include <memory>
#include <string>

namespace trajectory_algo
{{

class {name}OpParamAccess final : public IOpParamAccess
{{
public:
\tbool handlesKey(const std::string& key) const override;
\tbool read(
\t\tconst RobotInstruction::TrajectoryOpDescriptor& op,
\t\tconst TrajectoryOpParamField& field,
\t\tTrajectoryParamValue& out) const override;
\tbool write(
\t\tRobotInstruction::TrajectoryOpDescriptor& op,
\t\tconst TrajectoryOpParamField& field,
\t\tconst TrajectoryParamValue& in) const override;
}};

std::unique_ptr<IOpParamAccess> make{name}OpParamAccess();

}} // namespace trajectory_algo
""",
        encoding="utf-8",
    )
    (folder / f"{name}OpParamAccess.cpp").write_text(
        f"""#include "{name}OpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{{

bool {name}OpParamAccess::handlesKey(const std::string& key) const
{{
\t{handles}
}}

bool {name}OpParamAccess::read(
\tconst RobotInstruction::TrajectoryOpDescriptor& op,
\tconst TrajectoryOpParamField& field,
\tTrajectoryParamValue& out) const
{{
{chr(10).join(read_blocks) if read_blocks else "\t(void)op;\n\t(void)field;\n\t(void)out;"}
\treturn false;
}}

bool {name}OpParamAccess::write(
\tRobotInstruction::TrajectoryOpDescriptor& op,
\tconst TrajectoryOpParamField& field,
\tconst TrajectoryParamValue& in) const
{{
{chr(10).join(write_blocks) if write_blocks else "\t(void)op;\n\t(void)field;\n\t(void)in;"}
\treturn false;
}}

std::unique_ptr<IOpParamAccess> make{name}OpParamAccess()
{{
\treturn std::make_unique<{name}OpParamAccess>();
}}

}} // namespace trajectory_algo
""",
        encoding="utf-8",
    )


def write_json(res_dir: Path, spec: dict):
    import json

    fields = []
    for i, (key, label_en, label_zh, unit, default) in enumerate(spec["fields"]):
        fields.append(
            {
                "key": key,
                "type": "Double",
                "labelZh": label_zh,
                "unit": unit,
                "default": default,
                "order": i,
            }
        )
    doc = {
        "version": 1,
        "kind": spec["kind"],
        "schema": {"fields": fields},
        "defaults": {"scope": {"kind": 1}, "params": spec["json_params"]},
    }
    (res_dir / f"{spec['name']}.json").write_text(
        json.dumps(doc, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def main():
    for spec in ATOMIC_OPS:
        folder = BUILTINS / spec["name"]
        folder.mkdir(parents=True, exist_ok=True)
        write_op_h(folder, spec["name"])
        write_op_cpp(folder, spec)
        write_config(folder, spec["name"], spec["kind"])
        write_param_access(folder, spec)
        write_json(RES, spec)
        print(f"generated {spec['name']}")


if __name__ == "__main__":
    main()
