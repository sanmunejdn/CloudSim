# -*- coding: utf-8 -*-
"""Fix relative links in CloudSim first-party markdown guides."""
from __future__ import annotations

import re
from pathlib import Path

CLOUD = Path(r"d:/Project/VSprogram/CGAL5.5.2/CloudSim")

# (file_relative_to_cloud, list of (old, new) string replacements)
REPLACEMENTS: dict[str, list[tuple[str, str]]] = {}

def add(rel: str, pairs: list[tuple[str, str]]) -> None:
    REPLACEMENTS.setdefault(rel.replace("\\", "/"), []).extend(pairs)

# --- docs ---
add("docs/MODULE_DEVELOPER_GUIDES.md", [
    ("../../tools/ai-training/CONFIGURATION.md", "../tools/ai-training/CONFIGURATION.md"),
    ("../../tools/ai-training/README.md", "../tools/ai-training/README.md"),
])
add("docs/SOURCE_CONVENTIONS.md", [
    ("code_format_cleanup/ACCEPTANCE_code_format_cleanup.md",
     "_archive/code_format_cleanup/ACCEPTANCE_code_format_cleanup.md"),
])

# --- common archive doc remaps (applied after depth fixes where needed) ---
ARCHIVE_DOCS = [
    ("docs/template_brep_pointcloud_update.md", "docs/_archive/template_brep_pointcloud_update.md"),
    ("docs/mesh_surface_reconstruction.md", "docs/_archive/mesh_surface_reconstruction.md"),
    ("docs/spare_nonrigid_registration.md", "docs/_archive/spare_nonrigid_registration.md"),
    ("docs/trajectory_feature_ai.md", "docs/_archive/trajectory_feature_ai.md"),
    ("docs/ai_agent_runtime/", "docs/_archive/ai_agent_runtime/"),
    ("docs/mesh_reconstruction_optimization/", "docs/_archive/mesh_reconstruction_optimization/"),
    ("docs/vcglib_integration/", "docs/_archive/vcglib_integration/"),
    ("docs/三点圆弧指令/", "docs/_archive/三点圆弧指令/"),
    ("docs/三点圆弧指令/CONSENSUS_三点圆弧指令.md", "docs/_archive/三点圆弧指令/CONSENSUS_三点圆弧指令.md"),
    ("docs/外部轴联动求解/", "docs/_archive/外部轴联动求解/"),
    ("docs/外部轴联动求解/FINAL_外部轴联动求解.md", "docs/_archive/外部轴联动求解/FINAL_外部轴联动求解.md"),
    ("docs/外部轴类型拓宽/", "docs/_archive/外部轴类型拓宽/"),
    ("docs/外部轴类型拓宽/DESIGN_外部轴类型拓宽.md", "docs/_archive/外部轴类型拓宽/DESIGN_外部轴类型拓宽.md"),
    ("docs/机器人程序品牌导出/", "docs/_archive/机器人程序品牌导出/"),
    ("docs/code_format_cleanup/", "docs/_archive/code_format_cleanup/"),
]

# Depth: from src/<A>/<B>/file.md -> ../../../docs/
# Many files wrongly use ../../docs/

SRC_GUIDES = list((CLOUD / "src").rglob("DEVELOPER_GUIDE.md")) + list((CLOUD / "src").rglob("README.md"))
SRC_GUIDES = [p for p in SRC_GUIDES if "third_party" not in p.parts and "onecad_src" not in p.parts and "ported" not in p.parts]

changed_files = []

for path in SRC_GUIDES:
    rel = str(path.relative_to(CLOUD)).replace("\\", "/")
    text = path.read_text(encoding="utf-8-sig")
    orig = text

    # Fix ../../docs/ -> ../../../docs/ when file is under src/*/*/
    # But Plugins linking ../../tools is CORRECT (CloudSim/tools). Don't touch tools.
    # Pattern: ](../../docs/  -> ](../../../docs/
    text = re.sub(r"\]\(\.\./\.\./docs/", "](../../../docs/", text)
    # App/CloudSim wrong ../docs/
    if rel.startswith("src/App/"):
        text = text.replace("](../docs/", "](../../../docs/")
        text = text.replace("](../../docs/", "](../../../docs/")

    # Cross-module path fixes by location
    if rel.startswith("src/Data/Data/"):
        text = text.replace("](../CloudSimPluginHost/", "](../../UI/CloudSimPluginHost/")
        text = text.replace("](../PointCloudAlgorithm/", "](../../Geometry/PointCloudAlgorithm/")
        text = text.replace("](../VcgAlgorithms/", "](../../Geometry/VcgAlgorithms/")
        text = text.replace("](../BackendVisual/", "](../../UI/BackendVisual/")
        text = text.replace("](../Widget/", "](../../UI/Widget/")
        text = text.replace("](../inc/GeometryBackendOps.h)", "](inc/GeometryBackendOps.h)")
        text = text.replace("](../../docs/后端对象", "](../../../docs/后端对象")
        text = text.replace("](../../docs/_archive/", "](../../../docs/_archive/")

    if rel.startswith("src/Geometry/GeometryAlgorithm/"):
        text = text.replace("](../Data/inc/", "](../../Data/Data/inc/")
        text = text.replace("](../Data/DEVELOPER_GUIDE.md)", "](../../Data/Data/DEVELOPER_GUIDE.md)")

    if rel.startswith("src/Geometry/GeometryEngine/"):
        text = text.replace("](../RobotScene/", "](../../Robot/RobotScene/")
        text = text.replace("](../RobotUrdf/", "](../../Robot/RobotUrdf/")

    if rel.startswith("src/Geometry/PointCloudAlgorithm/"):
        text = text.replace("](../Data/inc/", "](../../Data/Data/inc/")
        text = text.replace("](../Data/DEVELOPER_GUIDE.md)", "](../../Data/Data/DEVELOPER_GUIDE.md)")

    if rel.startswith("src/Robot/RobotScene/"):
        text = text.replace("](../RobotWidget/", "](../../UI/RobotWidget/")
        text = text.replace("](../Widget/", "](../../UI/Widget/")
        text = text.replace("](../GeometryEngine/", "](../../Geometry/GeometryEngine/")
        text = text.replace("](../Geometry/GeometryAlgorithm/", "](../../Geometry/GeometryAlgorithm/")
        # ../../docs/机器人 -> should be ../../../docs/ after general fix; brand export was ../../docs
        text = text.replace("](../../docs/机器人", "](../../../docs/机器人")

    if rel.startswith("src/Robot/RobotUrdf/"):
        text = text.replace("](../GeometryEngine/", "](../../Geometry/GeometryEngine/")
        text = text.replace("](../Widget/", "](../../UI/Widget/")

    if rel.startswith("src/Robot/TrajectoryAlgorithmBuiltins/"):
        text = text.replace("](../UI/RobotWidget/", "](../../UI/RobotWidget/")
        text = text.replace("](../../docs/MODULE", "](../../../docs/MODULE")

    if rel.startswith("src/UI/BackendVisual/"):
        text = text.replace("](../Data/Data/", "](../../Data/Data/")
        text = text.replace("](../Host/CloudSimHost/", "](../../Host/CloudSimHost/")
        text = text.replace("](../Contracts/CloudSimCore/", "](../../Contracts/CloudSimCore/")

    if rel.startswith("src/UI/CloudSimPluginHost/"):
        text = text.replace("](../Data/Data/", "](../../Data/Data/")
        # tools from UI: ../../tools is CloudSim/tools OK
        # trajectory_feature already ../../../docs - will get archive remap

    if rel.startswith("src/UI/OsgWidgetCore/"):
        text = text.replace("](../Host/CloudSimHost/", "](../../Host/CloudSimHost/")

    if rel.startswith("src/UI/RobotWidget/"):
        text = text.replace("](../Robot/RobotScene/", "](../../Robot/RobotScene/")
        text = text.replace("](../RobotScene/", "](../../Robot/RobotScene/")
        text = text.replace("](../TrajectoryAlgorithm/", "](../../Robot/TrajectoryAlgorithm/")
        text = text.replace("](../Host/CloudSimHost/", "](../../Host/CloudSimHost/")
        text = text.replace("](../GeometryEngine/", "](../../Geometry/GeometryEngine/")
        text = text.replace("](../../docs/MODULE", "](../../../docs/MODULE")

    if rel.startswith("src/Plugins/CloudSimAiSDK/"):
        text = text.replace("](../App/CloudSim/", "](../../App/CloudSim/")
        text = text.replace("](../Plugins/PointNetPlugin/", "](../PointNetPlugin/")
        text = text.replace("](../../../docs/ai_agent_runtime/", "](../../../docs/_archive/ai_agent_runtime/")
        # ../../docs/trajectory -> ../../../docs after depth fix
        text = text.replace("](../../docs/trajectory_feature_ai.md)", "](../../../docs/_archive/trajectory_feature_ai.md)")

    if rel.startswith("src/Plugins/CloudSimLabelingSDK/"):
        text = text.replace("](../../LabelingPlugin/)", "](../LabelingPlugin/)")

    if rel.startswith("src/Plugins/CloudSimMeshTrajectorySDK/"):
        text = text.replace("](../UI/RobotWidget/", "](../../UI/RobotWidget/")

    # Apply archive remaps for ../../../docs/... patterns
    for old, new in ARCHIVE_DOCS:
        # after depth fix links look like ../../../docs/foo
        text = text.replace(f"](../../../{old})", f"](../../../{new})")
        text = text.replace(f"](../../{old})", f"](../../../{new})")  # leftover shallow

    # Avoid double _archive
    text = text.replace("docs/_archive/_archive/", "docs/_archive/")

    if text != orig:
        path.write_text(text, encoding="utf-8-sig", newline="\r\n")
        changed_files.append(rel)

# docs-level replacements
for rel, pairs in REPLACEMENTS.items():
    path = CLOUD / rel
    if not path.exists():
        print("missing", rel)
        continue
    text = path.read_text(encoding="utf-8-sig")
    orig = text
    for a, b in pairs:
        text = text.replace(a, b)
    if text != orig:
        path.write_text(text, encoding="utf-8-sig", newline="\r\n")
        changed_files.append(rel)

print(f"changed {len(changed_files)} files")
for f in changed_files:
    print(" ", f)
