#!/usr/bin/env python3
"""根据 TrajectoryAlgorithm.vcxproj 生成 .vcxproj.filters（UTF-8 BOM）"""
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1] / "src" / "Robot" / "TrajectoryAlgorithm"
vcx = ROOT / "TrajectoryAlgorithm.vcxproj"
filters = ROOT / "TrajectoryAlgorithm.vcxproj.filters"

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
    "      <UniqueIdentifier>{6E19DB17-114F-4732-94FD-5C613C7C6E45}</UniqueIdentifier>",
    "    </Filter>",
    '    <Filter Include="src">',
    "      <UniqueIdentifier>{63311DD2-D53A-425A-B170-45556AEEC27D}</UniqueIdentifier>",
    "    </Filter>",
    "  </ItemGroup>",
    "  <ItemGroup>",
]
for inc in sorted(includes):
    lines.append(f'    <ClInclude Include="{inc}">')
    lines.append("      <Filter>inc</Filter>")
    lines.append("    </ClInclude>")
lines.append("  </ItemGroup>")
lines.append("  <ItemGroup>")
for src in sorted(compiles):
    lines.append(f'    <ClCompile Include="{src}">')
    lines.append("      <Filter>src</Filter>")
    lines.append("    </ClCompile>")
lines.append("  </ItemGroup>")
lines.append("</Project>")
lines.append("")

# UTF-8 BOM，避免 VS 中文环境漏读 filters
filters.write_bytes(b"\xef\xbb\xbf" + "\n".join(lines).encode("utf-8"))
print(f"wrote {filters} ({len(includes)} headers, {len(compiles)} sources)")
