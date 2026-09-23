# -*- coding: utf-8 -*-
"""对照 CloudSimHost / CloudSimHostHeadless 的 ClCompile、ClInclude，防止网页端漏编共享源。

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
	}
)

# 路径片段：命中则视为仅桌面（与 basename 表互补）
DESKTOP_ONLY_PATH_SUBSTR = (
	"ViewportInteraction\\",
	"ViewportInteraction/",
)

# 仅 Headless
HEADLESS_ONLY_BASENAMES = frozenset(
	{
		"OsgWidgetSceneBridge_Headless.cpp",
	}
)

ITEM_RE = re.compile(
	r'<(ClCompile|ClInclude)\s+Include="([^"]+)"',
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
	return basename(path) in HEADLESS_ONLY_BASENAMES


def collect_items(vcxproj: Path) -> dict[str, set[str]]:
	text = vcxproj.read_text(encoding="utf-8")
	out: dict[str, set[str]] = {"ClCompile": set(), "ClInclude": set()}
	for m in ITEM_RE.finditer(text):
		tag, inc = m.group(1), norm(m.group(2))
		out[tag].add(inc)
	return out


def classify(
	desktop: dict[str, set[str]], headless: dict[str, set[str]]
) -> dict[str, list[str]]:
	"""返回需关注的漂移列表。"""
	result: dict[str, list[str]] = {
		"missing_in_headless_shared": [],
		"unexpected_in_headless_desktop_only": [],
		"missing_in_desktop_shared": [],
		"unexpected_in_desktop_headless_only": [],
	}
	for tag in ("ClCompile", "ClInclude"):
		d, h = desktop[tag], headless[tag]
		for p in sorted(d - h):
			if is_desktop_only(p) or is_headless_only(p):
				continue
			result["missing_in_headless_shared"].append(f"{tag}:{p}")
		for p in sorted(h - d):
			if is_headless_only(p):
				continue
			if is_desktop_only(p):
				result["unexpected_in_headless_desktop_only"].append(f"{tag}:{p}")
			else:
				result["missing_in_desktop_shared"].append(f"{tag}:{p}")
		for p in sorted(d):
			if is_headless_only(p):
				result["unexpected_in_desktop_headless_only"].append(f"{tag}:{p}")
	return result


def insert_item(text: str, tag: str, include: str) -> str:
	"""在同类 ItemGroup 中按字典序插入一行；找不到锚点则追加到第一个 Cl* ItemGroup。"""
	line = f'    <{tag} Include="{include}" />'
	if f'<{tag} Include="{include}"' in text:
		return text

	# 在最后一个同 tag 的 Include 行后插入（保持相对顺序：找字典序邻居）
	pattern = re.compile(rf'(    <{tag} Include="([^"]+)"[^/]*/>\s*\n)', re.IGNORECASE)
	matches = list(pattern.finditer(text))
	if not matches:
		# 退路：在 </ItemGroup> 前插入（第一个含 ClCompile/ClInclude 的组）
		m = re.search(
			rf'(<(?:ClCompile|ClInclude)\s+Include="[^"]+"[^>]*>.*?</ItemGroup>)',
			text,
			re.S | re.I,
		)
		if not m:
			raise RuntimeError(f"无法在 vcxproj 中定位 {tag} ItemGroup")
		# 简化：在文件末尾 Import 前插入新 ItemGroup
		insert = f"  <ItemGroup>\n{line}\n  </ItemGroup>\n"
		anchor = '  <Import Project="$(VCTargetsPath)\\Microsoft.Cpp.targets"'
		if anchor in text:
			return text.replace(anchor, insert + anchor, 1)
		return text + "\n" + insert

	# 找第一个 Include 路径 >= 新路径的位置前插；否则插在最后一个后
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
	"""把 missing_in_headless_shared 写入 Headless vcxproj。返回已写入项。"""
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
		"missing_in_headless_shared": "共享源仅在桌面（网页漏编）",
		"missing_in_desktop_shared": "共享源仅在网页（桌面漏编）",
		"unexpected_in_headless_desktop_only": "桌面专用却出现在 Headless",
		"unexpected_in_desktop_headless_only": "Headless 专用却出现在桌面",
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
		print("OK: CloudSimHost / CloudSimHostHeadless 共享源对齐")
	return problems


def main() -> int:
	ap = argparse.ArgumentParser(description="Check CloudSimHost vs Headless source lists")
	ap.add_argument(
		"--fix",
		action="store_true",
		help="将「共享源仅在桌面」自动写入 CloudSimHostHeadless.vcxproj",
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

	if not DESKTOP.is_file() or not HEADLESS.is_file():
		print("ERROR: vcxproj 未找到", file=sys.stderr)
		return 2

	desktop = collect_items(DESKTOP)
	headless = collect_items(HEADLESS)
	classified = classify(desktop, headless)

	if args.fix and classified["missing_in_headless_shared"]:
		written = fix_headless(classified["missing_in_headless_shared"])
		print(f"--fix: 已写入 Headless {len(written)} 项")
		for w in written:
			print(f"  + {w}")
		print("请接着执行:")
		print("  python scripts/generate_vcxproj_filters.py --sync --project CloudSimHostHeadless")
		headless = collect_items(HEADLESS)
		classified = classify(desktop, headless)

	n = print_report(classified)
	if n and not args.fix:
		print("\n修复建议: python scripts/check_host_headless_sources.py --fix")
		print("而后: python scripts/generate_vcxproj_filters.py --sync --project CloudSimHostHeadless")
		print("并 Debug|x64 + Release|x64 编译 CloudSimHostHeadless")
	return 1 if n else 0


if __name__ == "__main__":
	sys.exit(main())
