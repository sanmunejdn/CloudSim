# -*- coding: utf-8 -*-
"""从 CloudSimHost / Headless 提取共享 ClCompile/ClInclude 到 CloudSimHostShared.items.props。"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HOST_DIR = ROOT / "src" / "Host" / "CloudSimHost"
DESKTOP = HOST_DIR / "CloudSimHost.vcxproj"
HEADLESS = HOST_DIR / "CloudSimHostHeadless.vcxproj"
PROPS = HOST_DIR / "CloudSimHostShared.items.props"

ITEM_RE = re.compile(r'<(ClCompile|ClInclude)\s+Include="([^"]+)"([^>]*)/>')

DESKTOP_ONLY_BASENAMES = {
	"GraphicsWindowQt1.cpp",
	"QWidgetViewer.cpp",
	"ObjectTransformOperation.cpp",
	"RobotTcpDragTeachOperation.cpp",
	"MeshSectionPlaneEditOperation.cpp",
	"PointPickOperation.cpp",
	"LabelingPickOperation.cpp",
	"PolylinePickOperation.cpp",
	"MeshEdgeFacePickOperation.cpp",
	"OsgRenderViewAdapter.cpp",
	"OsgWidgetSceneBridge.cpp",
	"OsgWidget.cpp",
	"OsgWidgetBackendLoadController.cpp",
	"OsgWidgetCaptureController.cpp",
	"OsgWidgetColorController.cpp",
	"OsgWidgetCameraFocusController.cpp",
	"OsgWidgetGizmoController.cpp",
	"OsgWidgetImportController.cpp",
	"OsgWidgetMeshSectionPlane.cpp",
	"OsgWidgetPickAnnotationController.cpp",
	"OsgWidgetTcpTeach.cpp",
	"OsgWidgetTransformHierarchyController.cpp",
	"OsgWidgetPickEngine.cpp",
	"ViewportInteractionController.cpp",
	"PluginPropertyBindingSelfTest.cpp",
	"PluginPropertyBindingSelfTest.h",
	"ViewportGestureRecognizer.cpp",
	"QWidgetViewer.h",
	"OsgWidget.h",
	"GraphicsWindowQt1.h",
}
HEADLESS_ONLY_BASENAMES = {"OsgWidgetSceneBridge_Headless.cpp"}


def norm(p: str) -> str:
	return p.replace("/", "\\")


def basename(p: str) -> str:
	return Path(norm(p)).name


def collect_lines(path: Path) -> list[tuple[str, str, str, str]]:
	"""[(tag, include, rest, full_line_stripped)]"""
	out = []
	for line in path.read_text(encoding="utf-8").splitlines():
		m = ITEM_RE.search(line)
		if not m:
			continue
		out.append((m.group(1), norm(m.group(2)), m.group(3), line.strip()))
	return out


def main() -> int:
	d_items = collect_lines(DESKTOP)
	h_items = collect_lines(HEADLESS)
	d_map = {(t, i): (rest, line) for t, i, rest, line in d_items}
	h_map = {(t, i): (rest, line) for t, i, rest, line in h_items}
	shared_keys = sorted(set(d_map) & set(h_map))

	shared_xml: list[str] = []
	shared_set: set[tuple[str, str]] = set()
	for key in shared_keys:
		tag, inc = key
		bn = basename(inc)
		if bn in DESKTOP_ONLY_BASENAMES or bn in HEADLESS_ONLY_BASENAMES:
			continue
		if "ViewportInteraction\\" in inc and bn.endswith(".cpp"):
			# 视口交互 cpp 仅桌面
			if bn in DESKTOP_ONLY_BASENAMES:
				continue
		rest, _ = d_map[key]
		shared_xml.append(f'    <{tag} Include="{inc}"{rest} />')
		shared_set.add(key)

	PROPS.write_text(
		'<?xml version="1.0" encoding="utf-8"?>\r\n'
		'<Project xmlns="http://schemas.microsoft.com/developer/msbuild/2003">\r\n'
		"  <!-- Host 桌面/Headless 共享源；专用项留在各自 vcxproj -->\r\n"
		"  <ItemGroup>\r\n"
		+ "\r\n".join(shared_xml)
		+ "\r\n  </ItemGroup>\r\n"
		"</Project>\r\n",
		encoding="utf-8",
		newline="\r\n",
	)
	print(f"WROTE props shared={len(shared_xml)}")

	def strip_shared(vcx: Path) -> None:
		lines_out = []
		removed = 0
		for line in vcx.read_text(encoding="utf-8").splitlines(keepends=True):
			m = ITEM_RE.search(line)
			if m:
				key = (m.group(1), norm(m.group(2)))
				if key in shared_set:
					removed += 1
					continue
			lines_out.append(line)
		text = "".join(lines_out)
		imp = '  <Import Project="CloudSimHostShared.items.props" />\r\n'
		if "CloudSimHostShared.items.props" not in text:
			for anchor in (
				'  <Import Project="$(VCTargetsPath)\\Microsoft.Cpp.targets"',
				'  <Import Project="$(VCTargetsPath)/Microsoft.Cpp.targets"',
			):
				if anchor in text:
					text = text.replace(anchor, imp + anchor, 1)
					break
		vcx.write_text(text, encoding="utf-8", newline="\r\n")
		print(f"STRIP {vcx.name} removed={removed}")

	strip_shared(DESKTOP)
	strip_shared(HEADLESS)
	return 0


if __name__ == "__main__":
	sys.exit(main())
