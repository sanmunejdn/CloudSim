#!/usr/bin/env python3
from pathlib import Path
import re

vcxproj = Path(__file__).resolve().parents[1] / "src" / "Robot" / "TrajectoryAlgorithmBuiltins" / "TrajectoryAlgorithmBuiltins.vcxproj"
text = vcxproj.read_text(encoding="utf-8")

REMOVE = ["RecipeWeld", "RecipeGlue", "RecipeGrind"]
ADD = [
    "Resample", "OffsetAlongNormal", "OffsetLateral", "SmoothPose",
    "AssignBlend", "AssignSpeedZone", "Weave", "ReachabilityFilter", "ExternalAxisSearch",
]

for name in REMOVE:
    text = re.sub(rf'\s*<ClInclude Include="ops\\{name}\\[^"]+" />\s*\n', "", text)
    text = re.sub(rf'\s*<ClCompile Include="ops\\{name}\\[^"]+" />\s*\n', "", text)
    text = text.replace(f"ops/{name};", "")

include_dirs = text.split("AdditionalIncludeDirectories")[1][:800]
for name in ADD:
    if f"ops/{name};" not in text:
        text = text.replace(
            "ops/Retract;",
            f"ops/Retract;ops/{name};",
            2,
        )

new_includes = []
new_compiles = []
for name in ADD:
    if f"{name}Op.h" not in text:
        new_includes.append(f'    <ClInclude Include="ops\\{name}\\{name}Op.h" />\n')
        new_includes.append(f'    <ClInclude Include="ops\\{name}\\{name}OpConfig.h" />\n')
        new_includes.append(f'    <ClInclude Include="ops\\{name}\\{name}OpParamAccess.h" />\n')
        new_compiles.append(f'    <ClCompile Include="ops\\{name}\\{name}Op.cpp" />\n')
        new_compiles.append(f'    <ClCompile Include="ops\\{name}\\{name}OpConfig.cpp" />\n')
        new_compiles.append(f'    <ClCompile Include="ops\\{name}\\{name}OpParamAccess.cpp" />\n')

if "UnifiedTrajectoryPathMath.h" not in text:
    new_includes.insert(0, '    <ClInclude Include="inc\\UnifiedTrajectoryPathMath.h" />\n')
    new_includes.insert(0, '    <ClInclude Include="inc\\TrajectoryUnifiedScope.h" />\n')
    new_compiles.insert(0, '    <ClCompile Include="source\\UnifiedTrajectoryPathMath.cpp" />\n')
    new_compiles.insert(0, '    <ClCompile Include="source\\TrajectoryUnifiedScope.cpp" />\n')

if new_includes:
    text = text.replace(
        '    <ClInclude Include="inc\\TrajectoryOpPathApply.h" />\n',
        '    <ClInclude Include="inc\\TrajectoryOpPathApply.h" />\n' + "".join(new_includes),
    )
    text = text.replace(
        '    <ClCompile Include="source\\TrajectoryOpBuiltinsRegister.cpp" />\n',
        '    <ClCompile Include="source\\TrajectoryOpBuiltinsRegister.cpp" />\n' + "".join(new_compiles),
    )

vcxproj.write_text(text, encoding="utf-8")
print("updated vcxproj")
