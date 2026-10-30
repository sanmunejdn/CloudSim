# -*- coding: utf-8 -*-
"""Generate P0-P2 trajectory op boilerplate (Op/Config/JSON) and patch vcxproj."""
from __future__ import annotations

import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BUILTINS = ROOT / "src/Robot/TrajectoryAlgorithmBuiltins"
SCENE = ROOT / "src/Robot/RobotScene"
OPS = BUILTINS / "ops"
JSON_DIR = SCENE / "resource/trajectory/ops"
VCX = BUILTINS / "TrajectoryAlgorithmBuiltins.vcxproj"
SCENE_VCX = SCENE / "RobotScene.vcxproj"

# name, kindToken, zh, en, fields for JSON + paramFields snippets
# fields: list of dict key, type Double/Int, labelZh, labelEn, unit?, default, order
OPS_SPEC = [
    {
        "name": "ReversePath",
        "zh": "路径反向",
        "en": "Reverse Path",
        "fields": [],
        "comment": "反转作用域内点序",
    },
    {
        "name": "OrientToPath",
        "zh": "按路径定向",
        "en": "Orient To Path",
        "fields": [
            {
                "key": "orient.flipToolZ",
                "type": "Bool",
                "labelZh": "翻转工具Z",
                "labelEn": "Flip tool Z",
                "defaultBool": False,
                "order": 0,
            }
        ],
        "comment": "按弦切向+世界Z重建姿态",
    },
    {
        "name": "TrimPath",
        "zh": "弧长裁剪",
        "en": "Trim Path",
        "fields": [
            {
                "key": "trim.startMm",
                "type": "Double",
                "labelZh": "起点裁剪",
                "labelEn": "Trim start",
                "unit": "mm",
                "default": 0.0,
                "order": 0,
            },
            {
                "key": "trim.endMm",
                "type": "Double",
                "labelZh": "终点裁剪",
                "labelEn": "Trim end",
                "unit": "mm",
                "default": 0.0,
                "order": 1,
            },
        ],
        "comment": "按弧长去掉首尾",
    },
    {
        "name": "ShiftStart",
        "zh": "闭环起点平移",
        "en": "Shift Start",
        "fields": [
            {
                "key": "shift.startIndex1",
                "type": "Int",
                "labelZh": "新起点序号",
                "labelEn": "New start index",
                "defaultInt": 1,
                "order": 0,
            }
        ],
        "comment": "闭合路径旋转起点",
    },
    {
        "name": "InsertMid",
        "zh": "段中插点",
        "en": "Insert Midpoints",
        "fields": [
            {
                "key": "insert.passes",
                "type": "Int",
                "labelZh": "加密遍数",
                "labelEn": "Passes",
                "defaultInt": 1,
                "order": 0,
            }
        ],
        "comment": "每段中点插值加密",
    },
    {
        "name": "ClampToPlane",
        "zh": "贴平面",
        "en": "Clamp To Plane",
        "fields": [
            {
                "key": "clamp.axis",
                "type": "Enum",
                "labelZh": "锁定轴",
                "labelEn": "Axis",
                "enumValues": ["0", "1", "2"],
                "enumLabelsZh": ["X", "Y", "Z"],
                "enumLabelsEn": ["X", "Y", "Z"],
                "defaultInt": 2,
                "order": 0,
            },
            {
                "key": "clamp.valueMm",
                "type": "Double",
                "labelZh": "目标值",
                "labelEn": "Value",
                "unit": "mm",
                "default": 0.0,
                "order": 1,
            },
        ],
        "comment": "将坐标轴锁到给定值",
    },
    {
        "name": "ScalePath",
        "zh": "路径缩放",
        "en": "Scale Path",
        "fields": [
            {
                "key": "scale.factor",
                "type": "Double",
                "labelZh": "比例",
                "labelEn": "Factor",
                "default": 1.0,
                "order": 0,
            }
        ],
        "comment": "相对作用域首点缩放位置",
    },
    {
        "name": "SpeedRamp",
        "zh": "速度坡道",
        "en": "Speed Ramp",
        "fields": [
            {
                "key": "speed.startMmPerSec",
                "type": "Double",
                "labelZh": "起点速度",
                "labelEn": "Start speed",
                "unit": "mm/s",
                "default": 50.0,
                "order": 0,
            },
            {
                "key": "speed.endMmPerSec",
                "type": "Double",
                "labelZh": "终点速度",
                "labelEn": "End speed",
                "unit": "mm/s",
                "default": 100.0,
                "order": 1,
            },
        ],
        "comment": "作用域内速度线性插值",
    },
    {
        "name": "SpikeFilter",
        "zh": "尖点滤波",
        "en": "Spike Filter",
        "fields": [
            {
                "key": "spike.maxDetourRatio",
                "type": "Double",
                "labelZh": "绕行比阈值",
                "labelEn": "Max detour ratio",
                "default": 1.5,
                "order": 0,
            }
        ],
        "comment": "去掉相对弦线过长的尖点",
    },
]


def param_fields_cpp(fields: list) -> str:
    if not fields:
        return "\treturn {};"
    lines = ["\treturn {"]
    for f in fields:
        key = f["key"]
        en = f["labelEn"]
        zh = f["labelZh"]
        order = f.get("order", 0)
        if f["type"] == "Double":
            unit = f.get("unit", "")
            default = f.get("default", 0.0)
            lines.append(
                f'\t\tdoubleParamField("{key}", "{en}", "{zh}", "{unit}", -1e5, 1e5, 0.01, {default}, {order}),'
            )
        elif f["type"] == "Int":
            default = f.get("defaultInt", 0)
            lines.append(f'\t\tintParamField("{key}", "{en}", "{zh}", 0, 999999, {default}, {order}),')
        elif f["type"] == "Bool":
            default = "true" if f.get("defaultBool") else "false"
            lines.append(f'\t\tboolParamField("{key}", "{en}", "{zh}", {default}, {order}),')
        elif f["type"] == "Enum":
            ev = ", ".join(f'"{v}"' for v in f["enumValues"])
            ez = ", ".join(f'"{v}"' for v in f["enumLabelsZh"])
            ee = ", ".join(f'"{v}"' for v in f["enumLabelsEn"])
            default = f.get("defaultInt", 0)
            lines.append(
                f'\t\tenumParamField("{key}", "{en}", "{zh}", {{{ev}}}, {{{ez}}}, {{{ee}}}, {default}, {order}),'
            )
    lines.append("\t};")
    return "\n".join(lines)


def process_path_body(name: str) -> str:
    # Call into PathMath helpers; params read inline
    mapping = {
        "ReversePath": """\treversePathUnifiedInScope(traj, op.scope, ctx.program);
\treturn true;""",
        "OrientToPath": """\tconst bool flipZ = trajectoryParamBool(op.params, "orient.flipToolZ", false);
\torientToPathUnifiedInScope(traj, op.scope, ctx.program, flipZ);
\treturn true;""",
        "TrimPath": """\tconst double startMm = trajectoryParamDouble(op.params, "trim.startMm", 0.0);
\tconst double endMm = trajectoryParamDouble(op.params, "trim.endMm", 0.0);
\ttrimPathUnifiedInScope(traj, op.scope, ctx.program, startMm, endMm);
\treturn true;""",
        "ShiftStart": """\tconst int start1 = trajectoryParamInt(op.params, "shift.startIndex1", 1);
\tshiftStartUnifiedInScope(traj, op.scope, ctx.program, start1);
\treturn true;""",
        "InsertMid": """\tconst int passes = trajectoryParamInt(op.params, "insert.passes", 1);
\tinsertMidpointsUnifiedInScope(traj, op.scope, ctx.program, passes);
\treturn true;""",
        "ClampToPlane": """\tconst int axis = trajectoryParamInt(op.params, "clamp.axis", 2);
\tconst double valueMm = trajectoryParamDouble(op.params, "clamp.valueMm", 0.0);
\tclampToPlaneUnifiedInScope(traj, op.scope, ctx.program, axis, valueMm);
\treturn true;""",
        "ScalePath": """\tconst double factor = trajectoryParamDouble(op.params, "scale.factor", 1.0);
\tscalePathUnifiedInScope(traj, op.scope, ctx.program, factor);
\treturn true;""",
        "SpeedRamp": """\tconst double v0 = trajectoryParamDouble(op.params, "speed.startMmPerSec", 50.0);
\tconst double v1 = trajectoryParamDouble(op.params, "speed.endMmPerSec", 100.0);
\tspeedRampUnifiedInScope(traj, op.scope, ctx.program, v0, v1);
\treturn true;""",
        "SpikeFilter": """\tconst double ratio = trajectoryParamDouble(op.params, "spike.maxDetourRatio", 1.5);
\tspikeFilterUnifiedInScope(traj, op.scope, ctx.program, ratio);
\treturn true;""",
    }
    return mapping[name]


def validate_body(name: str) -> str:
    if name == "TrimPath":
        return """\tif (trajectoryParamDouble(op.params, "trim.startMm", 0.0) < 0.0 ||
\t\ttrajectoryParamDouble(op.params, "trim.endMm", 0.0) < 0.0)
\t{
\t\tif (errMsg)
\t\t{
\t\t\t*errMsg = "trim lengths must be >= 0";
\t\t}
\t\treturn false;
\t}
\treturn true;"""
    if name == "InsertMid":
        return """\tif (trajectoryParamInt(op.params, "insert.passes", 1) < 1)
\t{
\t\tif (errMsg)
\t\t{
\t\t\t*errMsg = "insert passes must be >= 1";
\t\t}
\t\treturn false;
\t}
\treturn true;"""
    if name == "ScalePath":
        return """\tif (std::abs(trajectoryParamDouble(op.params, "scale.factor", 1.0)) < 1e-9)
\t{
\t\tif (errMsg)
\t\t{
\t\t\t*errMsg = "scale factor must be non-zero";
\t\t}
\t\treturn false;
\t}
\treturn true;"""
    if name == "SpikeFilter":
        return """\tif (trajectoryParamDouble(op.params, "spike.maxDetourRatio", 1.5) < 1.0)
\t{
\t\tif (errMsg)
\t\t{
\t\t\t*errMsg = "detour ratio must be >= 1";
\t\t}
\t\treturn false;
\t}
\treturn true;"""
    return """\t(void)op;
\t(void)errMsg;
\treturn true;"""


def write_op(spec: dict) -> None:
    name = spec["name"]
    zh = spec["zh"]
    en = spec["en"]
    comment = spec["comment"]
    d = OPS / name
    d.mkdir(parents=True, exist_ok=True)

    guard = f"TRAJECTORYALGORITHMBUILTINS_{name.upper()}OP_H"
    (d / f"{name}Op.h").write_text(
        f"""#ifndef {guard}
#define {guard}

/// @file {name}Op.h
/// @brief {name}Op 接口

// {name}：{comment}
#include "ITrajectoryOp.h"

namespace trajectory_algo
{{
class {name}Op final : public ITrajectoryOp
{{
public:
\tRobotInstruction::TrajectoryOpKind kind() const override;
\tconst char* kindToken() const override {{ return "{name}"; }}
\tconst char* displayName(bool chinese) const override;
\tTrajectoryOpCapability capabilities() const override;
\tRobotInstruction::TrajectoryOpDescriptor
\tmakeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const override;
\tstd::vector<TrajectoryOpParamField> paramFields() const override;
\tbool validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const override;
\tstd::string formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, bool chinese) const override;
\tbool processPath(const RobotInstruction::TrajectoryOpDescriptor& op, RobotInstruction::UnifiedTrajectory& traj,
\t\t\t\t\t const TrajectoryOpExecutionContext& ctx, std::string* errMsg) const override;
}};

}} // namespace trajectory_algo

#endif // {guard}
""",
        encoding="utf-8",
    )

    need_cmath = name in ("ScalePath", "SpikeFilter", "TrimPath")
    includes = [
        f'#include "{name}Op.h"',
        "",
        '#include "TrajectoryOpFormat.h"',
        '#include "TrajectoryOpParamAccess.h"',
        '#include "TrajectoryOpParamsParse.h"',
        '#include "UnifiedTrajectoryPathMath.h"',
    ]
    if need_cmath:
        includes.append("")
        includes.append("#include <cmath>")

    (d / f"{name}Op.cpp").write_text(
        f"""/// @file {name}Op.cpp
/// @brief {name} 轨迹算子

// {name}：{comment}
{chr(10).join(includes)}

namespace trajectory_algo
{{
RobotInstruction::TrajectoryOpKind {name}Op::kind() const
{{
\treturn RobotInstruction::TrajectoryOpKind::{name};
}}

const char* {name}Op::displayName(const bool chinese) const
{{
\treturn chinese ? "{zh}" : "{en}";
}}

TrajectoryOpCapability {name}Op::capabilities() const
{{
\treturn TrajectoryOpCapability::None;
}}

RobotInstruction::TrajectoryOpDescriptor
{name}Op::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{{
\tRobotInstruction::TrajectoryOpDescriptor op{{}};
\top.kind = RobotInstruction::TrajectoryOpKind::{name};
\top.scope = defaultScope;
\tTrajectoryOpParamAccess::applyDefaults(op, *this);
\treturn op;
}}

std::vector<TrajectoryOpParamField> {name}Op::paramFields() const
{{
{param_fields_cpp(spec["fields"])}
}}

bool {name}Op::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{{
{validate_body(name)}
}}

std::string {name}Op::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{{
\t(void)op;
\treturn displayName(chinese);
}}

bool {name}Op::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
\t\t\t\t\t\t   RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
\t\t\t\t\t\t   std::string* errMsg) const
{{
\t(void)errMsg;
\tif (traj.points.empty())
\t{{
\t\treturn false;
\t}}
{process_path_body(name)}
}}

}} // namespace trajectory_algo
""",
        encoding="utf-8",
    )

    cfg_guard = f"TRAJECTORYALGORITHMBUILTINS_{name.upper()}OPCONFIG_H"
    (d / f"{name}OpConfig.h").write_text(
        f"""#ifndef {cfg_guard}
#define {cfg_guard}

/// @file {name}OpConfig.h
/// @brief {name}OpConfig 接口

#include "IOpParamConfig.h"

#include <memory>

namespace trajectory_algo
{{
std::unique_ptr<IOpParamConfig> make{name}OpConfig();

}} // namespace trajectory_algo

#endif // {cfg_guard}
""",
        encoding="utf-8",
    )
    (d / f"{name}OpConfig.cpp").write_text(
        f"""/// @file {name}OpConfig.cpp
/// @brief {name} 算子配置

#include "{name}OpConfig.h"

#include "TrajectoryOpConfigImpl.h"
#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{{
std::unique_ptr<IOpParamConfig> make{name}OpConfig()
{{
\treturn makeTrajectoryOpConfig(RobotInstruction::TrajectoryOpKind::{name}, "ops/{name}.json");
}}

}} // namespace trajectory_algo
""",
        encoding="utf-8",
    )

    # JSON
    schema_fields = []
    defaults_params = {}
    for f in spec["fields"]:
        item = {
            "key": f["key"],
            "type": f["type"],
            "labelZh": f["labelZh"],
            "order": f.get("order", 0),
        }
        if "unit" in f:
            item["unit"] = f["unit"]
        if f["type"] == "Double":
            item["default"] = f.get("default", 0.0)
            defaults_params[f["key"]] = f.get("default", 0.0)
        elif f["type"] == "Int":
            item["defaultInt"] = f.get("defaultInt", 0)
            defaults_params[f["key"]] = f.get("defaultInt", 0)
        elif f["type"] == "Bool":
            item["defaultBool"] = f.get("defaultBool", False)
            defaults_params[f["key"]] = f.get("defaultBool", False)
        elif f["type"] == "Enum":
            item["enumValues"] = f["enumValues"]
            item["enumLabelsZh"] = f["enumLabelsZh"]
            item["enumLabelsEn"] = f["enumLabelsEn"]
            item["defaultInt"] = f.get("defaultInt", 0)
            defaults_params[f["key"]] = f.get("defaultInt", 0)
        schema_fields.append(item)

    j = {
        "version": 1,
        "kind": name,
        "schema": {"fields": schema_fields},
        "defaults": {"scope": {"kind": 1}, "params": defaults_params},
    }
    JSON_DIR.mkdir(parents=True, exist_ok=True)
    (JSON_DIR / f"{name}.json").write_text(json.dumps(j, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def patch_vcxproj() -> None:
    text = VCX.read_text(encoding="utf-8")
    # AdditionalIncludeDirectories: insert ops/Name after ops/Resample
    for spec in OPS_SPEC:
        name = spec["name"]
        token = f"ops/{name};"
        if token not in text:
            text = text.replace("ops/Resample;", f"ops/Resample;{token}")
        inc = f'<ClInclude Include="ops\\{name}\\{name}Op.h"><Filter>src</Filter></ClInclude>\n'
        inc2 = f'    <ClInclude Include="ops\\{name}\\{name}OpConfig.h"><Filter>src</Filter></ClInclude>\n'
        if f"ops\\{name}\\{name}Op.h" not in text:
            text = text.replace(
                '<ClInclude Include="ops\\Resample\\ResampleOpConfig.h"><Filter>src</Filter></ClInclude>\n',
                '<ClInclude Include="ops\\Resample\\ResampleOpConfig.h"><Filter>src</Filter></ClInclude>\n'
                + inc
                + inc2,
            )
        cc = f'<ClCompile Include="ops\\{name}\\{name}Op.cpp"><Filter>src</Filter></ClCompile>\n'
        cc2 = f'    <ClCompile Include="ops\\{name}\\{name}OpConfig.cpp"><Filter>src</Filter></ClCompile>\n'
        if f"ops\\{name}\\{name}Op.cpp" not in text:
            text = text.replace(
                '<ClCompile Include="ops\\Resample\\ResampleOpConfig.cpp"><Filter>src</Filter></ClCompile>\n',
                '<ClCompile Include="ops\\Resample\\ResampleOpConfig.cpp"><Filter>src</Filter></ClCompile>\n'
                + cc
                + cc2,
            )
    VCX.write_text(text, encoding="utf-8")

    scene = SCENE_VCX.read_text(encoding="utf-8")
    for spec in OPS_SPEC:
        name = spec["name"]
        line = f'    <None Include="resource\\trajectory\\ops\\{name}.json" />\n'
        if f"ops\\{name}.json" not in scene:
            scene = scene.replace(
                '    <None Include="resource\\trajectory\\ops\\Resample.json" />\n',
                '    <None Include="resource\\trajectory\\ops\\Resample.json" />\n' + line,
            )
    SCENE_VCX.write_text(scene, encoding="utf-8")


def main() -> None:
    for spec in OPS_SPEC:
        write_op(spec)
        print("wrote", spec["name"])
    patch_vcxproj()
    print("patched vcxproj")


if __name__ == "__main__":
    main()
