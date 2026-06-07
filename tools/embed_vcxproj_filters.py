#!/usr/bin/env python3
"""为 vcxproj 中每个 ClInclude/ClCompile 写入 <Filter> 子节点（inc/src）"""
import xml.etree.ElementTree as ET
from pathlib import Path

NS = "http://schemas.microsoft.com/developer/msbuild/2003"
ET.register_namespace("", NS)


def embed(project_dir: Path, project_name: str) -> None:
    vcx_path = project_dir / f"{project_name}.vcxproj"
    tree = ET.parse(vcx_path)
    root = tree.getroot()
    changed = 0
    for tag, filt in (("ClInclude", "inc"), ("ClCompile", "src")):
        for item in root.findall(f"{{{NS}}}ItemGroup/{{{NS}}}{tag}"):
            include = item.attrib.get("Include", "")
            if not include:
                continue
            expected = filt if tag == "ClInclude" else "src"
            if include.startswith("source\\") or include.startswith("ops\\"):
                expected = "src"
            if include.startswith("inc\\"):
                expected = "inc"
            node = item.find(f"{{{NS}}}Filter")
            if node is None:
                node = ET.SubElement(item, f"{{{NS}}}Filter")
                changed += 1
            if node.text != expected:
                node.text = expected
                changed += 1
    if changed:
        tree.write(vcx_path, encoding="utf-8", xml_declaration=True)
    print(f"{project_name}: updated {changed} filter nodes")


def main() -> None:
    base = Path(__file__).resolve().parents[1] / "src" / "Robot"
    embed(base / "TrajectoryAlgorithmBuiltins", "TrajectoryAlgorithmBuiltins")


if __name__ == "__main__":
    main()
