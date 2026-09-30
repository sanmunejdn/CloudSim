# -*- coding: utf-8 -*-
"""对照 CloudSimHostCore / Viewport / Host / Headless 源清单，防止共享源漂移。

三方模型:
  - Core: 共享源（唯一）
  - Viewport: DESKTOP_ONLY（桌面 OSG 视口）
  - Headless: HEADLESS_ONLY 风味源
  - Host DLL: 内联桌面源应为空（仅 pch）

用法（在 CloudSim/ 根目录）:
  python scripts/check_host_headless_sources.py
  python scripts/check_host_headless_sources.py --fix
  python scripts/check_host_headless_sources.py --list-allow

退出码: 0 对齐；1 有漂移（未 --fix 或 fix 后仍有无法自动处理的项）。
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

# Windows 控制台避免中文乱码
if hasattr(sys.stdout, "reconfigure"):
	sys.stdout.reconfigure(encoding="utf-8", errors="replace")
	sys.stderr.reconfigure(encoding="utf-8", errors="replace")

ROOT = Path(__file__).resolve().parents[1]
HOST_DIR = ROOT / "src" / "Host" / "CloudSimHost"
DESKTOP = HOST_DIR / "CloudSimHost.vcxproj"
VIEWPORT = HOST_DIR / "CloudSimHostDesktopViewport.vcxproj"
HEADLESS = HOST_DIR / "CloudSimHostHeadless.vcxproj"
CORE = HOST_DIR / "CloudSimHostCore.vcxproj"
SHARED_PROPS_IMPORT = "CloudSimHostShared.items.props"

# 仅桌面 Viewport：OSG 视口 / Widget UI / 自测；勿进 Headless / Host 内联 / Core
DESKTOP_ONLY_BASENAMES = frozenset(
	{
		"GraphicsWindowQt1.cpp",
		"QWidgetViewer.cpp",
		"ObjectTransformOperation.cpp",
		"RobotTcpDragTeachOperation.cpp",
		"MeshSectionPlaneEditOperation.cpp",
		"PointPickOperation.cpp",
		"LabelingPickOperation.cpp",
		"PolylinePickOperation.cpp",
		"MeshEdgeFacePickOperation.cpp",
		"SelectionOperation.cpp",
		"ViewportGestureRecognizer.cpp",
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
		"OsgWidgetViewportInteractionHost.cpp",
		"OsgWidgetPickEngine.cpp",
		"ViewportInteractionController.cpp",
		"PluginPropertyBindingSelfTest.cpp",
		"PluginPropertyBindingSelfTest.h",
		"HostOsgFlavorHooks_Desktop.cpp",
	}
)

# 路径片段：命中则视为仅桌面（与 basename 表互补）
DESKTOP_ONLY_PATH_SUBSTR = (
	"ViewportInteraction\\",
	"ViewportInteraction/",
	"inc\\osg\\OsgWidget.h",
	"inc\\osg\\QWidgetViewer.h",
	"inc/osg/OsgWidget.h",
	"inc/osg/QWidgetViewer.h",
)

# 仅 Headless
HEADLESS_ONLY_BASENAMES = frozenset(
	{
		"OsgWidgetSceneBridge_Headless.cpp",
		"HostOsgFlavorHooks_Headless.cpp",
	}
)

# 两 DLL / Viewport / Core 各自编译 pch，不参与风味对齐
FLAVOR_NEUTRAL_BASENAMES = frozenset({"pch.cpp"})

HEADLESS_ONLY_PATH_SUBSTR = (
	"inc\\headless_stub\\",
	"inc/headless_stub/",
)

ITEM_RE = re.compile(
	r'<(ClCompile|ClInclude|QtMoc)\s+Include="([^"]+)"',
	re.IGNORECASE,
)


def norm(path: str) -> str:
	return path.replace("/", "\\")


def basename(path: str) -> str:
	return Path(norm(path)).name


def is_desktop_only(path: str) -> bool:
	p = norm(path)
	if basename(p) in DESKTOP_ONLY_BASENAMES:
		return True
	return any(s in p for s in DESKTOP_ONLY_PATH_SUBSTR)


def is_headless_only(path: str) -> bool:
	p = norm(path)
	if basename(p) in FLAVOR_NEUTRAL_BASENAMES:
		return False
	if basename(p) in HEADLESS_ONLY_BASENAMES:
		return True
	return any(s in p for s in HEADLESS_ONLY_PATH_SUBSTR)


def is_flavor_neutral(path: str) -> bool:
	return basename(norm(path)) in FLAVOR_NEUTRAL_BASENAMES


def collect_inline_items(vcxproj: Path) -> dict[str, set[str]]:
	"""只扫 vcxproj 本体的直列项，不跟随 Import。"""
	out: dict[str, set[str]] = {"ClCompile": set(), "ClInclude": set(), "QtMoc": set()}
	text = vcxproj.read_text(encoding="utf-8")
	for m in ITEM_RE.finditer(text):
		out[m.group(1)].add(norm(m.group(2)))
	return out


def imports_deleted_shared_props(vcxproj: Path) -> bool:
	return SHARED_PROPS_IMPORT in vcxproj.read_text(encoding="utf-8")


def classify(
	desktop_inline: dict[str, set[str]],
	viewport_inline: dict[str, set[str]],
	headless_inline: dict[str, set[str]],
	core_shared: dict[str, set[str]],
) -> dict[str, list[str]]:
	"""返回需关注的漂移列表。"""
	result: dict[str, list[str]] = {
		"host_inline_desktop_only": [],
		"missing_in_viewport": [],
		"unexpected_in_viewport": [],
		"unexpected_in_headless_desktop_only": [],
		"unexpected_in_desktop_headless_only": [],
		"desktop_only_in_core": [],
		"shared_inline_in_host": [],
		"shared_inline_in_headless": [],
		"still_imports_shared_props": [],
		"viewport_has_wholearchive": [],
	}
	for tag in ("ClCompile", "ClInclude", "QtMoc"):
		d, v, h = desktop_inline[tag], viewport_inline[tag], headless_inline[tag]
		for p in sorted(d):
			if is_flavor_neutral(p):
				continue
			if is_desktop_only(p):
				result["host_inline_desktop_only"].append(f"{tag}:{p}")
			elif is_headless_only(p):
				result["unexpected_in_desktop_headless_only"].append(f"{tag}:{p}")
			elif p in core_shared[tag]:
				result["shared_inline_in_host"].append(f"{tag}:{p}")
		for p in sorted(v):
			if is_flavor_neutral(p):
				continue
			if is_headless_only(p):
				result["unexpected_in_viewport"].append(f"{tag}:{p}")
			elif not is_desktop_only(p) and basename(p) != "cloudsim_viewport_global.h":
				if tag == "ClCompile":
					result["unexpected_in_viewport"].append(f"{tag}:{p}")
		for p in sorted(h):
			if is_flavor_neutral(p):
				continue
			if is_desktop_only(p):
				result["unexpected_in_headless_desktop_only"].append(f"{tag}:{p}")
			elif p in core_shared[tag]:
				result["shared_inline_in_headless"].append(f"{tag}:{p}")
		for p in sorted(core_shared[tag]):
			if is_desktop_only(p):
				result["desktop_only_in_core"].append(f"{tag}:{p}")

	viewport_all = (
		viewport_inline["ClCompile"] | viewport_inline["ClInclude"] | viewport_inline["QtMoc"]
	)
	viewport_basenames = {basename(x) for x in viewport_all}
	for name in sorted(DESKTOP_ONLY_BASENAMES):
		if name == "PluginPropertyBindingSelfTest.h":
			if name not in viewport_basenames:
				result["missing_in_viewport"].append(f"ClInclude:{name}")
			continue
		if name.endswith(".cpp") and name not in viewport_basenames:
			result["missing_in_viewport"].append(f"ClCompile:{name}")
	for moc_name in ("OsgWidget.h", "QWidgetViewer.h"):
		if not any(basename(x) == moc_name for x in viewport_inline["QtMoc"]):
			result["missing_in_viewport"].append(f"QtMoc:{moc_name}")

	for path, label in (
		(DESKTOP, "CloudSimHost.vcxproj"),
		(VIEWPORT, "CloudSimHostDesktopViewport.vcxproj"),
		(HEADLESS, "CloudSimHostHeadless.vcxproj"),
		(CORE, "CloudSimHostCore.vcxproj"),
	):
		if path.is_file() and imports_deleted_shared_props(path):
			result["still_imports_shared_props"].append(label)
	if VIEWPORT.is_file():
		text = VIEWPORT.read_text(encoding="utf-8")
		# 只查链接选项，避免注释误报
		if re.search(r"<AdditionalOptions>[^<]*WHOLEARCHIVE", text, re.I):
			result["viewport_has_wholearchive"].append("CloudSimHostDesktopViewport.vcxproj")
	return result


def insert_item(text: str, tag: str, include: str) -> str:
	"""在同类 ItemGroup 中按字典序插入一行；找不到锚点则追加到第一个 Cl* ItemGroup。"""
	line = f'    <{tag} Include="{include}" />'
	if f'<{tag} Include="{include}"' in text:
		return text
	pattern = re.compile(rf'(    <{tag} Include="([^"]+)"[^/]*/>\s*\n)', re.IGNORECASE)
	matches = list(pattern.finditer(text))
	if not matches:
		insert = f"  <ItemGroup>\n{line}\n  </ItemGroup>\n"
		anchor = '  <Import Project="$(VCTargetsPath)\\Microsoft.Cpp.targets"'
		if anchor in text:
			return text.replace(anchor, insert + anchor, 1)
		return text + "\n" + insert
	place_before = None
	for m in matches:
		if m.group(2).replace("/", "\\") >= include:
			place_before = m
			break
	if place_before is not None:
		return text[: place_before.start()] + line + "\n" + text[place_before.start() :]
	last = matches[-1]
	return text[: last.end()] + line + "\n" + text[last.end() :]


def fix_headless(missing_basenames: list[str]) -> list[str]:
	"""兼容旧 --fix：仅处理 Headless 缺风味源（本脚本三方模型下较少使用）。"""
	return []


def print_report(classified: dict[str, list[str]]) -> int:
	labels = {
		"host_inline_desktop_only": "Host DLL 仍内联桌面专用源（应迁 Viewport）",
		"missing_in_viewport": "桌面专用未出现在 Viewport",
		"unexpected_in_viewport": "Viewport 出现非桌面专用项",
		"unexpected_in_headless_desktop_only": "桌面专用却出现在 Headless",
		"unexpected_in_desktop_headless_only": "Headless 专用却出现在 Host",
		"desktop_only_in_core": "桌面专用出现在 Core",
		"shared_inline_in_host": "共享源直列在 Host vcxproj（应只在 Core）",
		"shared_inline_in_headless": "共享源直列在 Headless vcxproj（应只在 Core）",
		"still_imports_shared_props": "仍 Import 已删除的 CloudSimHostShared.items.props",
		"viewport_has_wholearchive": "Viewport 禁止 /WHOLEARCHIVE",
	}
	problems = 0
	for key, title in labels.items():
		items = classified.get(key) or []
		if not items:
			continue
		problems += len(items)
		print(f"\n[{title}] ({len(items)})")
		for x in items:
			print(f"  {x}")
	if problems == 0:
		print("OK: Core / Viewport / Host / Headless 三方源对齐")
	return problems


def main() -> int:
	ap = argparse.ArgumentParser(description="Check Core / Viewport / Host / Headless source lists")
	ap.add_argument(
		"--fix",
		action="store_true",
		help="（保留）旧 Headless 补齐；三方模型下请手工迁源到 Viewport",
	)
	ap.add_argument(
		"--list-allow",
		action="store_true",
		help="打印桌面专用 / Headless 专用 allowlist",
	)
	args = ap.parse_args()

	if args.list_allow:
		print("DESKTOP_ONLY_BASENAMES:")
		for x in sorted(DESKTOP_ONLY_BASENAMES):
			print(f"  {x}")
		print("HEADLESS_ONLY_BASENAMES:")
		for x in sorted(HEADLESS_ONLY_BASENAMES):
			print(f"  {x}")
		return 0

	missing = [p for p in (DESKTOP, VIEWPORT, HEADLESS, CORE) if not p.is_file()]
	if missing:
		print("ERROR: vcxproj 未找到:", ", ".join(str(p) for p in missing), file=sys.stderr)
		return 2

	desktop_inline = collect_inline_items(DESKTOP)
	viewport_inline = collect_inline_items(VIEWPORT)
	headless_inline = collect_inline_items(HEADLESS)
	core_shared = collect_inline_items(CORE)
	classified = classify(desktop_inline, viewport_inline, headless_inline, core_shared)

	if args.fix:
		print("--fix: 三方模型请将 DESKTOP_ONLY 迁入 CloudSimHostDesktopViewport.vcxproj（不自动改 Host）")

	n = print_report(classified)
	return 1 if n else 0


if __name__ == "__main__":
	sys.exit(main())
