from pathlib import Path
import re

def strip_param_access(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    text2 = re.sub(
        r'\s*<ClInclude Include="ops\\[^"]*ParamAccess[^"]*">.*?</ClInclude>\s*',
        "\n",
        text,
    )
    text2 = re.sub(
        r'\s*<ClCompile Include="ops\\[^"]*ParamAccess[^"]*">.*?</ClCompile>\s*',
        "\n",
        text2,
    )
    text2 = re.sub(
        r'\s*<ClInclude Include="inc\\IOpParamAccess.h">.*?</ClInclude>\s*',
        "\n",
        text2,
    )
    if text2 != text:
        path.write_text(text2, encoding="utf-8")
        print("updated", path)


def add_params_parse(path: Path) -> None:
    text = path.read_text(encoding="utf-8")
    if "TrajectoryOpParamsParse.h" not in text:
        text = text.replace(
            '<ClInclude Include="inc\\TrajectoryOpParamAccess.h">',
            '<ClInclude Include="inc\\TrajectoryOpParamsParse.h">\n      <Filter>inc</Filter>\n    </ClInclude>\n    <ClInclude Include="inc\\TrajectoryOpParamAccess.h">',
        )
    if "TrajectoryOpParamsParse.cpp" not in text:
        text = text.replace(
            '<ClCompile Include="source\\TrajectoryOpParamAccess.cpp">',
            '<ClCompile Include="source\\TrajectoryOpParamsParse.cpp">\n      <Filter>source</Filter>\n    </ClCompile>\n    <ClCompile Include="source\\TrajectoryOpParamAccess.cpp">',
        )
    path.write_text(text, encoding="utf-8")
    print("patched", path)


root = Path(r"d:\Project\VSprogram\CGAL5.5.2\CloudSim\src\Robot")
for rel in [
    "TrajectoryAlgorithmBuiltins/TrajectoryAlgorithmBuiltins.vcxproj",
    "TrajectoryAlgorithmBuiltins/TrajectoryAlgorithmBuiltins.vcxproj.filters",
    "TrajectoryAlgorithm/TrajectoryAlgorithm.vcxproj",
    "TrajectoryAlgorithm/TrajectoryAlgorithm.vcxproj.filters",
]:
    strip_param_access(root / rel)

for rel in [
    "TrajectoryAlgorithm/TrajectoryAlgorithm.vcxproj",
    "TrajectoryAlgorithm/TrajectoryAlgorithm.vcxproj.filters",
]:
    add_params_parse(root / rel)
