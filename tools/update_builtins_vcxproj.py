#!/usr/bin/env python3
from pathlib import Path

vcxproj = Path(__file__).resolve().parents[1] / "src" / "Robot" / "TrajectoryAlgorithmBuiltins" / "TrajectoryAlgorithmBuiltins.vcxproj"
text = vcxproj.read_text(encoding="utf-8")

ops = [
    "Translate", "Rotate", "Mirror", "Delete", "Duplicate", "Reorder",
    "RecipeWeld", "RecipeGlue", "RecipeGrind", "Approach", "Retract",
]

new_includes = [
    '    <ClInclude Include="inc\\TrajectoryOpConfigImpl.h" />\n',
]
new_compiles = [
    '    <ClCompile Include="source\\TrajectoryOpConfigImpl.cpp" />\n',
]
for name in ops:
    new_includes.append(f'    <ClInclude Include="ops\\{name}\\{name}OpConfig.h" />\n')
    new_includes.append(f'    <ClInclude Include="ops\\{name}\\{name}OpParamAccess.h" />\n')
    new_compiles.append(f'    <ClCompile Include="ops\\{name}\\{name}OpConfig.cpp" />\n')
    new_compiles.append(f'    <ClCompile Include="ops\\{name}\\{name}OpParamAccess.cpp" />\n')

if "TrajectoryOpConfigImpl.h" not in text:
    text = text.replace(
        '    <ClInclude Include="inc\\TrajectoryOpFormat.h" />\n',
        '    <ClInclude Include="inc\\TrajectoryOpFormat.h" />\n' + "".join(new_includes),
    )
    text = text.replace(
        '    <ClCompile Include="source\\TrajectoryOpFormat.cpp" />\n',
        '    <ClCompile Include="source\\TrajectoryOpFormat.cpp" />\n' + "".join(new_compiles),
    )

if "inc;" not in text.split("AdditionalIncludeDirectories")[1][:200]:
    text = text.replace(
        "AdditionalIncludeDirectories>inc;",
        "AdditionalIncludeDirectories>inc;../TrajectoryAlgorithmBuiltins/inc;",
        2,
    )

vcxproj.write_text(text, encoding="utf-8")
print("updated TrajectoryAlgorithmBuiltins.vcxproj")

algo = Path(__file__).resolve().parents[1] / "src" / "Robot" / "TrajectoryAlgorithm" / "TrajectoryAlgorithm.vcxproj"
atext = algo.read_text(encoding="utf-8")
add_h = [
    '    <ClInclude Include="inc\\IOpParamConfig.h" />\n',
    '    <ClInclude Include="inc\\IOpParamAccess.h" />\n',
    '    <ClInclude Include="inc\\TrajectoryOpConfigRegistry.h" />\n',
    '    <ClInclude Include="inc\\TrajectoryParamJsonIo.h" />\n',
]
add_cpp = [
    '    <ClCompile Include="source\\TrajectoryOpConfigRegistry.cpp" />\n',
    '    <ClCompile Include="source\\TrajectoryParamJsonIo.cpp" />\n',
]
if "IOpParamConfig.h" not in atext:
    atext = atext.replace(
        '    <ClInclude Include="inc\\TrajectoryOpParamAccess.h" />\n',
        '    <ClInclude Include="inc\\TrajectoryOpParamAccess.h" />\n' + "".join(add_h),
    )
    atext = atext.replace(
        '    <ClCompile Include="source\\TrajectoryOpParamAccess.cpp" />\n',
        '    <ClCompile Include="source\\TrajectoryOpParamAccess.cpp" />\n' + "".join(add_cpp),
    )
algo.write_text(atext, encoding="utf-8")
print("updated TrajectoryAlgorithm.vcxproj")
