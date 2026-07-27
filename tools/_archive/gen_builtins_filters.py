#!/usr/bin/env python3
"""根据 TrajectoryAlgorithmBuiltins.vcxproj 生成 .vcxproj.filters"""
import re
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "src" / "Robot" / "TrajectoryAlgorithmBuiltins"
vcx = ROOT / "TrajectoryAlgorithmBuiltins.vcxproj"
filters = ROOT / "TrajectoryAlgorithmBuiltins.vcxproj.filters"

NS = "http://schemas.microsoft.com/developer/msbuild/2003"
ET.register_namespace("", NS)

tree = ET.parse(vcx)
root = tree.getroot()

includes = [e.attrib["Include"] for e in root.findall(f"{{{NS}}}ItemGroup/{{{NS}}}ClInclude")]
compiles = [e.attrib["Include"] for e in root.findall(f"{{{NS}}}ItemGroup/{{{NS}}}ClCompile")]

lines = [
    '<?xml version="1.0" encoding="utf-8"?>',
    f'<Project xmlns="{NS}" ToolsVersion="4.0">',
    "  <ItemGroup>",
    '    <Filter Include="inc">',
    "      <UniqueIdentifier>{FFC815F7-4DD5-4C57-95BE-2D8233DD46EF}</UniqueIdentifier>",
    "    </Filter>",
    '    <Filter Include="src">',
    "      <UniqueIdentifier>{D29EF1AA-A1F1-4D5F-98B3-F8A58031525F}</UniqueIdentifier>",
    "    </Filter>",
    "  </ItemGroup>",
    "  <ItemGroup>",
]
for inc in includes:
    lines.append(f'    <ClInclude Include="{inc}">')
    lines.append("      <Filter>inc</Filter>")
    lines.append("    </ClInclude>")
lines.append("  </ItemGroup>")
lines.append("  <ItemGroup>")
for src in compiles:
    lines.append(f'    <ClCompile Include="{src}">')
    lines.append("      <Filter>src</Filter>")
    lines.append("    </ClCompile>")
lines.append("  </ItemGroup>")
lines.append("</Project>")
lines.append("")

filters.write_bytes(b"\xef\xbb\xbf" + "\n".join(lines).encode("utf-8"))
print(f"wrote {filters} ({len(includes)} headers, {len(compiles)} sources)")
