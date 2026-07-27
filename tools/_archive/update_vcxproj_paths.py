#!/usr/bin/env python3
"""One-shot path updater after src/ domain migration."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# (project_dir_suffix, list of (old, new) replacements) — order matters (longer first)
RULES: dict[str, list[tuple[str, str]]] = {
    "src/Infra/RunLogger": [
        ("../../bin/", "../../../bin/"),
    ],
    "src/Geometry/GeometryEngine": [
        ("../../OSG3.6.5", "../../../OSG3.6.5"),
        ("../../bin/", "../../../bin/"),
    ],
    "src/Geometry/PointCloudAlgorithm": [
        ("../../bin/", "../../../bin/"),
    ],
    "src/Robot/RobotKinematics": [
        ("../../bin/", "../../../bin/"),
    ],
    "src/Data/Data": [
        ("../RunLogger/", "../../Infra/RunLogger/"),
        ("../PointCloudAlgorithm/", "../../Geometry/PointCloudAlgorithm/"),
        ("..\\PointCloudAlgorithm\\", "..\\..\\Geometry\\PointCloudAlgorithm\\"),
        ("..\\RunLogger\\", "..\\..\\Infra\\RunLogger\\"),
        ("../../bin/", "../../../bin/"),
        ('Include="..\\PointCloudAlgorithm\\', 'Include="..\\..\\Geometry\\PointCloudAlgorithm\\'),
        ('Include="..\\RunLogger\\', 'Include="..\\..\\Infra\\RunLogger\\'),
    ],
    "src/Robot/RobotUrdf": [
        ("../GeometryEngine/", "../../Geometry/GeometryEngine/"),
        ("../BackendVisual/", "../../UI/BackendVisual/"),
        ("../Data/", "../../Data/Data/"),
        ("../RunLogger/", "../../Infra/RunLogger/"),
        ("..\\GeometryEngine\\", "..\\..\\Geometry\\GeometryEngine\\"),
        ("..\\BackendVisual\\", "..\\..\\UI\\BackendVisual\\"),
        ("..\\Data\\", "..\\..\\Data\\Data\\"),
        ("..\\RunLogger\\", "..\\..\\Infra\\RunLogger\\"),
        ("../../OSG3.6.5", "../../../OSG3.6.5"),
        ("../../bin/", "../../../bin/"),
    ],
    "src/Robot/RobotScene": [
        ("../GeometryEngine/", "../../Geometry/GeometryEngine/"),
        ("../RobotUrdf/", "../RobotUrdf/"),  # same Robot folder — keep
        ("../Data/", "../../Data/Data/"),
        ("../RobotKinematics/", "../RobotKinematics/"),
        ("../RunLogger/", "../../Infra/RunLogger/"),
        ("..\\GeometryEngine\\", "..\\..\\Geometry\\GeometryEngine\\"),
        ("..\\Data\\", "..\\..\\Data\\Data\\"),
        ("..\\RunLogger\\", "..\\..\\Infra\\RunLogger\\"),
        ("../../OSG3.6.5", "../../../OSG3.6.5"),
        ("../../bin/", "../../../bin/"),
    ],
    "src/UI/BackendVisual": [
        ("../GeometryEngine/", "../../Geometry/GeometryEngine/"),
        ("../Data/", "../../Data/Data/"),
        ("..\\GeometryEngine\\", "..\\..\\Geometry\\GeometryEngine\\"),
        ("..\\Data\\", "..\\..\\Data\\Data\\"),
        ("../../OSG3.6.5", "../../../OSG3.6.5"),
        ("../../bin/", "../../../bin/"),
    ],
    "src/UI/OsgWidgetCore": [
        ("../Data/", "../../Data/Data/"),
        ("../BackendVisual/", "../BackendVisual/"),
        ("../RunLogger/", "../../Infra/RunLogger/"),
        ("..\\Data\\", "..\\..\\Data\\Data\\"),
        ("..\\RunLogger\\", "..\\..\\Infra\\RunLogger\\"),
        ("../../OSG3.6.5", "../../../OSG3.6.5"),
        ("../../bin/", "../../../bin/"),
    ],
    "src/UI/Widget": [
        ("../CloudSimPluginSDK/", "../../Plugins/CloudSimPluginSDK/"),
        ("../AiBackend/", "../../AI/AiBackend/"),
        ("../GeometryEngine/", "../../Geometry/GeometryEngine/"),
        ("../RunLogger/", "../../Infra/RunLogger/"),
        ("../Data/", "../../Data/Data/"),
        ("../RobotKinematics/", "../../Robot/RobotKinematics/"),
        ("../RobotUrdf/", "../../Robot/RobotUrdf/"),
        ("../RobotScene/", "../../Robot/RobotScene/"),
        ("..\\CloudSimPluginSDK\\", "..\\..\\Plugins\\CloudSimPluginSDK\\"),
        ("..\\RobotKinematics\\", "..\\..\\Robot\\RobotKinematics\\"),
        ("..\\RobotUrdf\\", "..\\..\\Robot\\RobotUrdf\\"),
        ("..\\RobotScene\\", "..\\..\\Robot\\RobotScene\\"),
        ("..\\GeometryEngine\\", "..\\..\\Geometry\\GeometryEngine\\"),
        ("..\\RunLogger\\", "..\\..\\Infra\\RunLogger\\"),
        ("../../bin/", "../../../bin/"),
    ],
    "src/UI/RobotWidget": [
        ("../OsgWidgetCore/", "../OsgWidgetCore/"),
        ("../BackendVisual/", "../BackendVisual/"),
        ("../RobotScene/", "../../Robot/RobotScene/"),
        ("../RobotUrdf/", "../../Robot/RobotUrdf/"),
        ("../RobotKinematics/", "../../Robot/RobotKinematics/"),
        ("../GeometryEngine/", "../../Geometry/GeometryEngine/"),
        ("../Data/", "../../Data/Data/"),
        ("../RunLogger/", "../../Infra/RunLogger/"),
        ("..\\RobotScene\\", "..\\..\\Robot\\RobotScene\\"),
        ("..\\RobotUrdf\\", "..\\..\\Robot\\RobotUrdf\\"),
        ("..\\RobotKinematics\\", "..\\..\\Robot\\RobotKinematics\\"),
        ("..\\GeometryEngine\\", "..\\..\\Geometry\\GeometryEngine\\"),
        ("..\\Data\\", "..\\..\\Data\\Data\\"),
        ("..\\RunLogger\\", "..\\..\\Infra\\RunLogger\\"),
        ("../../bin/", "../../../bin/"),
    ],
    "src/UI/AiWidget": [
        ("../AiBackend/", "../../AI/AiBackend/"),
        ("..\\AiBackend\\", "..\\..\\AI\\AiBackend\\"),
        ("../../bin/", "../../../bin/"),
    ],
    "src/UI/CloudSimPluginHost": [
        ("../CloudSimPluginSDK/", "../../Plugins/CloudSimPluginSDK/"),
        ("../Data/", "../../Data/Data/"),
        ("../RunLogger/", "../../Infra/RunLogger/"),
        ("../GeometryEngine/", "../../Geometry/GeometryEngine/"),
        ("../RobotUrdf/", "../../Robot/RobotUrdf/"),
        ("../RobotScene/", "../../Robot/RobotScene/"),
        ("../RobotKinematics/", "../../Robot/RobotKinematics/"),
        ("../AiBackend/", "../../AI/AiBackend/"),
        ("..\\CloudSimPluginSDK\\", "..\\..\\Plugins\\CloudSimPluginSDK\\"),
        ("..\\Data\\", "..\\..\\Data\\Data\\"),
        ("..\\RunLogger\\", "..\\..\\Infra\\RunLogger\\"),
        ("../../bin/", "../../../bin/"),
    ],
    "src/AI/AiBackend": [
        ("../Data/", "../../Data/Data/"),
        ("..\\Data\\", "..\\..\\Data\\Data\\"),
        ("../../bin/", "../../../bin/"),
    ],
    "src/App/CloudSim": [
        ("../Widget/", "../../UI/Widget/"),
        ("../RobotKinematics/", "../../Robot/RobotKinematics/"),
        ("../RobotUrdf/", "../../Robot/RobotUrdf/"),
        ("../RobotScene/", "../../Robot/RobotScene/"),
        ("../Data/", "../../Data/Data/"),
        ("../GeometryEngine/", "../../Geometry/GeometryEngine/"),
        ("../RunLogger/", "../../Infra/RunLogger/"),
        ("../../bin/", "../../../bin/"),
    ],
    "src/Plugins/CloudSimPluginSDK": [],
}


def apply_rules(text: str, rules: list[tuple[str, str]]) -> str:
    for old, new in rules:
        text = text.replace(old, new)
    return text


def main() -> None:
    for suffix, rules in RULES.items():
        proj_dir = ROOT / suffix.replace("/", "\\") if False else ROOT / Path(*suffix.split("/"))
        for name in ("*.vcxproj", "*.vcxproj.filters"):
            for path in proj_dir.glob(name):
                original = path.read_text(encoding="utf-8")
                updated = apply_rules(original, rules)
                if updated != original:
                    path.write_text(updated, encoding="utf-8", newline="\r\n")
                    print(f"updated {path.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
