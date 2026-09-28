# -*- coding: utf-8 -*-
"""对照 CloudSimHostCore / 两 DLL 的源清单，防止共享源漂移或漏链 Core。

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
HEADLESS = HOST_DIR / "CloudSimHostHeadless.vcxproj"
CORE = HOST_DIR / "CloudSimHostCore.vcxproj"
SHARED_ITEMS = HOST_DIR / "CloudSimHostShared.items.props"
SHARED_IMPORT = "CloudSimHostShared.items.props"

# 仅桌面：OSG 视口 / Widget UI / 自测；勿强制进 Headless
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

# 两 DLL 与 Core 各自编译 pch，不参与风味对齐
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


def collect_items(vcxproj: Path) -> dict[str, set[str]]:
	"""含相对 Import 的 .props / .items.props 中的 ItemGroup。"""
	out: dict[str, set[str]] = {"ClCompile": set(), "ClInclude": set(), "QtMoc": set()}
	seen: set[Path] = set()

	def walk(path: Path) -> None:
		path = path.resolve()
		if path in seen or not path.exists():
			return
		seen.add(path)
		text = path.read_text(encoding="utf-8")
		for m in ITEM_RE.finditer(text):
			tag, inc = m.group(1), norm(m.group(2))
			out[tag].add(inc)
		for m in re.finditer(r'<Import\s+Project="([^"]+)"', text, re.I):
			proj = m.group(1)
			if "$(" in proj:
				continue
			if not re.search(r"\.(props|items\.props)$", proj, re.I):
				continue
			walk(path.parent / proj)

	walk(vcxproj)
	return out


def collect_inline_items(vcxproj: Path) -> dict[str, set[str]]:
	"""只扫 vcxproj 本体的直列项，不跟随 Import。"""
	out: dict[str, set[str]] = {"ClCompile": set(), "ClInclude": set(), "QtMoc": set()}
	text = vcxproj.read_text(encoding="utf-8")
	for m in ITEM_RE.finditer(text):
		out[m.group(1)].add(norm(m.group(2)))
	return out


def imports_shared_props(vcxproj: Path) -> bool:
	return SHARED_IMPORT in vcxproj.read_text(encoding="utf-8")


def classify(
	desktop_inline: dict[str, set[str]],
	headless_inline: dict[str, set[str]],
	shared: dict[str, set[str]],
	core: dict[str, set[str]],
) -> dict[str, list[str]]:
	"""返回需关注的漂移列表。"""
	result: dict[str, list[str]] = {
		"core_missing_shared": [],
		"core_extra_vs_shared": [],
		"missing_in_headless_flavor": [],
		"unexpected_in_headless_desktop_only": [],
		"missing_in_desktop_flavor": [],
		"unexpected_in_desktop_headless_only": [],
		"shared_inline_in_desktop": [],
		"shared_inline_in_headless": [],
		"dll_still_imports_shared": [],
	}
	for tag in ("ClCompile", "ClInclude"):
		s, c = shared[tag], core[tag]
		for p in sorted(s - c):
			result["core_missing_shared"].append(f"{tag}:{p}")
		for p in sorted(c - s):
			if basename(p) == "pch.cpp":
				continue
			result["core_extra_vs_shared"].append(f"{tag}:{p}")
	for tag in ("ClCompile", "ClInclude", "QtMoc"):
		d, h = desktop_inline[tag], headless_inline[tag]
		for p in sorted(d - h):
			if is_flavor_neutral(p) or is_desktop_only(p) or is_headless_only(p):
				continue
			result["missing_in_headless_flavor"].append(f"{tag}:{p}")
		for p in sorted(h - d):
			if is_flavor_neutral(p) or is_headless_only(p):
				continue
			if is_desktop_only(p):
				result["unexpected_in_headless_desktop_only"].append(f"{tag}:{p}")
			else:
				result["missing_in_desktop_flavor"].append(f"{tag}:{p}")
		for p in sorted(d):
			if is_flavor_neutral(p):
				continue
			if is_headless_only(p):
				result["unexpected_in_desktop_headless_only"].append(f"{tag}:{p}")
	for tag in ("ClCompile", "ClInclude"):
		for p in sorted(desktop_inline[tag] & shared[tag]):
			result["shared_inline_in_desktop"].append(f"{tag}:{p}")
		for p in sorted(headless_inline[tag] & shared[tag]):
			result["shared_inline_in_headless"].append(f"{tag}:{p}")
	if imports_shared_props(DESKTOP):
		result["dll_still_imports_shared"].append("CloudSimHost.vcxproj")
	if imports_shared_props(HEADLESS):
		result["dll_still_imports_shared"].append("CloudSimHostHeadless.vcxproj")
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


def fix_headless(missing: list[str]) -> list[str]:
	"""把 missing_in_headless_flavor 写入 Headless vcxproj。返回已写入项。"""
	if not missing:
		return []
	text = HEADLESS.read_text(encoding="utf-8")
	written: list[str] = []
	for entry in missing:
		tag, path = entry.split(":", 1)
		text = insert_item(text, tag, path)
		written.append(entry)
	HEADLESS.write_text(text, encoding="utf-8", newline="\n")
	return written


def print_report(classified: dict[str, list[str]]) -> int:
	labels = {
		"core_missing_shared": "CloudSimHostCore 缺 items.props 中的共享项",
		"core_extra_vs_shared": "CloudSimHostCore 多出 items.props 未列的项",
		"missing_in_headless_flavor": "风味源仅在桌面 DLL（网页漏编）",
		"missing_in_desktop_flavor": "风味源仅在网页 DLL（桌面漏编）",
		"unexpected_in_headless_desktop_only": "桌面专用却出现在 Headless",
		"unexpected_in_desktop_headless_only": "Headless 专用却出现在桌面",
		"shared_inline_in_desktop": "共享源直列在桌面 vcxproj（应只在 Core/items.props）",
		"shared_inline_in_headless": "共享源直列在 Headless vcxproj（应只在 Core/items.props）",
		"dll_still_imports_shared": "DLL 仍 Import CloudSimHostShared.items.props",
	}
	problems = 0
	for key, title in labels.items():
		items = classified[key]
		if not items:
			continue
		problems += len(items)
		print(f"\n[{title}] ({len(items)})")
		for x in items:
			print(f"  {x}")
	if problems == 0:
		print("OK: CloudSimHostCore / 两 DLL 风味源对齐")
	return problems


def main() -> int:
	ap = argparse.ArgumentParser(description="Check CloudSimHostCore vs Host DLL source lists")
	ap.add_argument(
		"--fix",
		action="store_true",
		help="将「风味源仅在桌面」自动写入 CloudSimHostHeadless.vcxproj",
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

	if not DESKTOP.is_file() or not HEADLESS.is_file() or not CORE.is_file():
		print("ERROR: vcxproj 未找到（需 CloudSimHost / Headless / Core）", file=sys.stderr)
		return 2

	desktop_inline = collect_inline_items(DESKTOP)
	headless_inline = collect_inline_items(HEADLESS)
	shared = collect_items(SHARED_ITEMS)
	core = collect_items(CORE)
	classified = classify(desktop_inline, headless_inline, shared, core)

	if args.fix and classified["missing_in_headless_flavor"]:
		written = fix_headless(classified["missing_in_headless_flavor"])
		print(f"--fix: 已写入 Headless {len(written)} 项")
		for w in written:
			print(f"  + {w}")
		print("请接着执行:")
		print("  python scripts/generate_vcxproj_filters.py --sync --project CloudSimHostHeadless")
		headless_inline = collect_inline_items(HEADLESS)
		classified = classify(desktop_inline, headless_inline, shared, core)

	n = print_report(classified)
	if classified["missing_in_headless_flavor"] and not args.fix:
		print("\n修复建议: python scripts/check_host_headless_sources.py --fix")
		print("而后: python scripts/generate_vcxproj_filters.py --sync --project CloudSimHostHeadless")
		print("并 Debug|x64 + Release|x64 编译 CloudSimHostHeadless")
	return 1 if n else 0


if __name__ == "__main__":
	sys.exit(main())
