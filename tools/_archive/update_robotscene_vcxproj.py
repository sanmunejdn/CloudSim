#!/usr/bin/env python3
"""确保 RobotScene.vcxproj 含 RawTrajectoryMath（inc/source），不再引用 raw/ops。"""
from pathlib import Path

vcx = Path(__file__).resolve().parents[1] / "src" / "Robot" / "RobotScene" / "RobotScene.vcxproj"
text = vcx.read_text(encoding="utf-8")

# 移除遗留 raw/ops 引用
text = text.replace(";raw/ops/common", "")
text = text.replace("raw\\ops\\common\\RawTrajectoryMath.h", "inc\\RawTrajectoryMath.h")
text = text.replace("raw\\ops\\common\\RawTrajectoryMath.cpp", "source\\RawTrajectoryMath.cpp")

legacy_raw_cpp = [
    "raw\\ops\\FrameFromPath\\FrameFromPathRawOp.cpp",
    "raw\\ops\\Resample\\ResampleRawOp.cpp",
    "raw\\ops\\OffsetAlongNormal\\OffsetAlongNormalRawOp.cpp",
    "raw\\ops\\OffsetLateral\\OffsetLateralRawOp.cpp",
    "raw\\ops\\SmoothPose\\SmoothPoseRawOp.cpp",
    "raw\\ops\\AssignBlend\\AssignBlendRawOp.cpp",
    "raw\\ops\\AssignSpeedZone\\AssignSpeedZoneRawOp.cpp",
    "raw\\ops\\Weave\\WeaveRawOp.cpp",
    "raw\\ops\\InsertApproachRetract\\InsertApproachRetractRawOp.cpp",
    "raw\\ops\\ReachabilityFilter\\ReachabilityFilterRawOp.cpp",
    "raw\\ops\\ExternalAxisSearch\\ExternalAxisSearchRawOp.cpp",
]
for line in legacy_raw_cpp:
    text = text.replace(f'    <ClCompile Include="{line}" />\n', "")

if "inc\\RawTrajectoryMath.h" not in text:
    text = text.replace(
        '    <ClInclude Include="inc\\RawTrajectory.h" />\n',
        '    <ClInclude Include="inc\\RawTrajectory.h" />\n'
        '    <ClInclude Include="inc\\RawTrajectoryMath.h" />\n',
    )

if "source\\RawTrajectoryMath.cpp" not in text:
    text = text.replace(
        '    <ClCompile Include="source\\RawTrajectory.cpp" />\n',
        '    <ClCompile Include="source\\RawTrajectory.cpp" />\n'
        '    <ClCompile Include="source\\RawTrajectoryMath.cpp" />\n',
    )

text = text.replace(
    "AdditionalIncludeDirectories>inc;raw/ops;raw/ops/common;",
    "AdditionalIncludeDirectories>inc;",
)

vcx.write_text(text, encoding="utf-8")
print("updated RobotScene.vcxproj (no raw/ops)")
