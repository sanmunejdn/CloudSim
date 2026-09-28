# -*- coding: utf-8 -*-
"""P0-1: 将 Host 借编的 Widget/PluginHost 源码物理迁入 CloudSimHost。

用法（CloudSim 根）:
  python scripts/migrate_host_borrowed_sources.py --dry-run
  python scripts/migrate_host_borrowed_sources.py
"""
from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HOST = ROOT / "src" / "Host" / "CloudSimHost"
WIDGET = ROOT / "src" / "UI" / "Widget"
PLUGIN = ROOT / "src" / "UI" / "CloudSimPluginHost"

# Widget 侧迁入 Host/inc|source/osg 的相对路径（相对 Widget 根）
WIDGET_MOVES: list[tuple[str, str]] = [
	# headers
	("inc/OsgWidget.h", "inc/osg/OsgWidget.h"),
	("inc/OsgWidgetBackendLoadController.h", "inc/osg/OsgWidgetBackendLoadController.h"),
	("inc/OsgWidgetCameraFocusController.h", "inc/osg/OsgWidgetCameraFocusController.h"),
	("inc/OsgWidgetCaptureController.h", "inc/osg/OsgWidgetCaptureController.h"),
	("inc/OsgWidgetColorController.h", "inc/osg/OsgWidgetColorController.h"),
	("inc/OsgWidgetGizmoController.h", "inc/osg/OsgWidgetGizmoController.h"),
	("inc/OsgWidgetImportController.h", "inc/osg/OsgWidgetImportController.h"),
	("inc/OsgWidgetPickAnnotationController.h", "inc/osg/OsgWidgetPickAnnotationController.h"),
	("inc/OsgWidgetSceneBridge.h", "inc/osg/OsgWidgetSceneBridge.h"),
	("inc/OsgWidgetTransformHierarchyController.h", "inc/osg/OsgWidgetTransformHierarchyController.h"),
	("inc/QWidgetViewer.h", "inc/osg/QWidgetViewer.h"),
	("inc/GraphicsWindowQt1.h", "inc/osg/GraphicsWindowQt1.h"),
	("inc/ObjectTransformOperation.h", "inc/osg/ObjectTransformOperation.h"),
	("inc/RobotTcpDragTeachOperation.h", "inc/osg/RobotTcpDragTeachOperation.h"),
	("inc/MeshSectionPlaneEditOperation.h", "inc/osg/MeshSectionPlaneEditOperation.h"),
	("inc/PointPickOperation.h", "inc/osg/PointPickOperation.h"),
	("inc/LabelingPickOperation.h", "inc/osg/LabelingPickOperation.h"),
	("inc/PolylinePickOperation.h", "inc/osg/PolylinePickOperation.h"),
	("inc/MeshEdgeFacePickOperation.h", "inc/osg/MeshEdgeFacePickOperation.h"),
	("inc/BackendSceneDocumentFacade.h", "inc/osg/BackendSceneDocumentFacade.h"),
	("inc/BackendFollowReverseIndex.h", "inc/osg/BackendFollowReverseIndex.h"),
	("inc/ViewportGestureRecognizer.h", "inc/osg/ViewportGestureRecognizer.h"),
	("inc/ViewportInteraction/OsgWidgetPickEngine.h", "inc/osg/ViewportInteraction/OsgWidgetPickEngine.h"),
	("inc/ViewportInteraction/ViewportHit.h", "inc/osg/ViewportInteraction/ViewportHit.h"),
	("inc/ViewportInteraction/ViewportInteractionController.h", "inc/osg/ViewportInteraction/ViewportInteractionController.h"),
	# sources
	("source/OsgWidget.cpp", "source/osg/OsgWidget.cpp"),
	("source/OsgWidgetBackendLoadController.cpp", "source/osg/OsgWidgetBackendLoadController.cpp"),
	("source/OsgWidgetCameraFocusController.cpp", "source/osg/OsgWidgetCameraFocusController.cpp"),
	("source/OsgWidgetCaptureController.cpp", "source/osg/OsgWidgetCaptureController.cpp"),
	("source/OsgWidgetColorController.cpp", "source/osg/OsgWidgetColorController.cpp"),
	("source/OsgWidgetGizmoController.cpp", "source/osg/OsgWidgetGizmoController.cpp"),
	("source/OsgWidgetImportController.cpp", "source/osg/OsgWidgetImportController.cpp"),
	("source/OsgWidgetPickAnnotationController.cpp", "source/osg/OsgWidgetPickAnnotationController.cpp"),
	("source/OsgWidgetSceneBridge.cpp", "source/osg/OsgWidgetSceneBridge.cpp"),
	("source/OsgWidgetTransformHierarchyController.cpp", "source/osg/OsgWidgetTransformHierarchyController.cpp"),
	("source/OsgWidgetMeshSectionPlane.cpp", "source/osg/OsgWidgetMeshSectionPlane.cpp"),
	("source/OsgWidgetTcpTeach.cpp", "source/osg/OsgWidgetTcpTeach.cpp"),
	("source/QWidgetViewer.cpp", "source/osg/QWidgetViewer.cpp"),
	("source/GraphicsWindowQt1.cpp", "source/osg/GraphicsWindowQt1.cpp"),
	("source/ObjectTransformOperation.cpp", "source/osg/ObjectTransformOperation.cpp"),
	("source/RobotTcpDragTeachOperation.cpp", "source/osg/RobotTcpDragTeachOperation.cpp"),
	("source/MeshSectionPlaneEditOperation.cpp", "source/osg/MeshSectionPlaneEditOperation.cpp"),
	("source/PointPickOperation.cpp", "source/osg/PointPickOperation.cpp"),
	("source/LabelingPickOperation.cpp", "source/osg/LabelingPickOperation.cpp"),
	("source/PolylinePickOperation.cpp", "source/osg/PolylinePickOperation.cpp"),
	("source/MeshEdgeFacePickOperation.cpp", "source/osg/MeshEdgeFacePickOperation.cpp"),
	("source/BackendSceneDocumentFacade.cpp", "source/osg/BackendSceneDocumentFacade.cpp"),
	("source/BackendFollowReverseIndex.cpp", "source/osg/BackendFollowReverseIndex.cpp"),
	("source/ViewportGestureRecognizer.cpp", "source/osg/ViewportGestureRecognizer.cpp"),
	("source/ViewportInteraction/OsgWidgetPickEngine.cpp", "source/osg/ViewportInteraction/OsgWidgetPickEngine.cpp"),
	("source/ViewportInteraction/ViewportInteractionController.cpp", "source/osg/ViewportInteraction/ViewportInteractionController.cpp"),
]

# 路径替换：长前缀优先，避免被短前缀截断
PATH_REPLACEMENTS = [
	("..\\..\\UI\\Widget\\source\\ViewportInteraction\\", "source\\osg\\ViewportInteraction\\"),
	("..\\..\\UI\\Widget\\source\\", "source\\osg\\"),
	("..\\..\\UI\\Widget\\inc\\ViewportInteraction\\", "inc\\osg\\ViewportInteraction\\"),
	("..\\..\\UI\\Widget\\inc\\", "inc\\osg\\"),
	("..\\..\\UI\\CloudSimPluginHost\\source\\", "source\\pluginhost\\"),
	("..\\..\\UI\\CloudSimPluginHost\\inc\\", "inc\\pluginhost\\"),
	("../../UI/Widget/source/ViewportInteraction/", "source/osg/ViewportInteraction/"),
	("../../UI/Widget/source/", "source/osg/"),
	("../../UI/Widget/inc/ViewportInteraction/", "inc/osg/ViewportInteraction/"),
	("../../UI/Widget/inc/", "inc/osg/"),
	("../../UI/CloudSimPluginHost/source/", "source/pluginhost/"),
	("../../UI/CloudSimPluginHost/inc/", "inc/pluginhost/"),
]

CONSUMER_INCLUDE_REPLACEMENTS = [
	# 其它工程 AdditionalIncludeDirectories 中的 PluginHost 路径
	("../../UI/CloudSimPluginHost/inc/Ai", "../../Host/CloudSimHost/inc/pluginhost/Ai"),
	("../../UI/CloudSimPluginHost/inc", "../../Host/CloudSimHost/inc/pluginhost"),
	("../CloudSimPluginHost/inc/Ai", "../../Host/CloudSimHost/inc/pluginhost/Ai"),
	("../CloudSimPluginHost/inc", "../../Host/CloudSimHost/inc/pluginhost"),
	("..\\..\\UI\\CloudSimPluginHost\\inc\\Ai", "..\\..\\Host\\CloudSimHost\\inc\\pluginhost\\Ai"),
	("..\\..\\UI\\CloudSimPluginHost\\inc", "..\\..\\Host\\CloudSimHost\\inc\\pluginhost"),
	("..\\CloudSimPluginHost\\inc\\Ai", "..\\..\\Host\\CloudSimHost\\inc\\pluginhost\\Ai"),
	("..\\CloudSimPluginHost\\inc", "..\\..\\Host\\CloudSimHost\\inc\\pluginhost"),
]


def git_mv(src: Path, dst: Path, dry: bool) -> None:
	dst.parent.mkdir(parents=True, exist_ok=True)
	if dry:
		print(f"MV {src.relative_to(ROOT)} -> {dst.relative_to(ROOT)}")
		return
	if not src.exists():
		print(f"SKIP missing {src.relative_to(ROOT)}", file=sys.stderr)
		return
	if dst.exists():
		print(f"SKIP exists {dst.relative_to(ROOT)}", file=sys.stderr)
		return
	r = subprocess.run(
		["git", "mv", str(src), str(dst)],
		cwd=ROOT,
		capture_output=True,
		text=True,
	)
	if r.returncode != 0:
		# 非 git 跟踪时降级为普通移动
		shutil.move(str(src), str(dst))
		print(f"MOVE(fallback) {src.relative_to(ROOT)} -> {dst.relative_to(ROOT)}")
	else:
		print(f"GITMV {src.relative_to(ROOT)} -> {dst.relative_to(ROOT)}")


def move_widget(dry: bool) -> None:
	for src_rel, dst_rel in WIDGET_MOVES:
		git_mv(WIDGET / src_rel, HOST / dst_rel, dry)


def move_pluginhost(dry: bool) -> None:
	"""整树迁入 Host/inc|source/pluginhost，保留相对结构。"""
	for sub in ("inc", "source"):
		src_root = PLUGIN / sub
		if not src_root.is_dir():
			continue
		for path in sorted(src_root.rglob("*")):
			if not path.is_file():
				continue
			rel = path.relative_to(src_root)
			dst = HOST / f"{sub}/pluginhost" / rel
			git_mv(path, dst, dry)


def rewrite_vcxproj(path: Path, dry: bool) -> None:
	text = path.read_text(encoding="utf-8")
	orig = text
	for old, new in PATH_REPLACEMENTS:
		text = text.replace(old, new)
	# AdditionalIncludeDirectories：PluginHost 旧路径 → 本地
	text = text.replace(
		"../../UI/CloudSimPluginHost/inc/Ai",
		"inc/pluginhost/Ai",
	)
	text = text.replace(
		"../../UI/CloudSimPluginHost/inc",
		"inc/pluginhost",
	)
	# 保留 Widget/inc（Widget 仍有 MainWindow 等），追加 osg 子目录
	needle = "../../UI/Widget/inc;"
	if needle in text and "inc/osg;" not in text and "inc\\osg;" not in text:
		text = text.replace(needle, "inc/osg;inc/pluginhost;inc/pluginhost/Ai;" + needle)
	if text == orig:
		print(f"NOCHANGE {path.relative_to(ROOT)}")
		return
	if dry:
		print(f"WOULD_PATCH {path.relative_to(ROOT)}")
		return
	path.write_text(text, encoding="utf-8", newline="\r\n")
	print(f"PATCHED {path.relative_to(ROOT)}")


def clean_widget_vcxproj(dry: bool) -> None:
	path = WIDGET / "Widget.vcxproj"
	text = path.read_text(encoding="utf-8")
	orig = text
	patterns = [
		r'\s*<ClInclude Include="inc\\OsgWidgetSceneBridge\.h" />\r?\n',
		r'\s*<ClInclude Include="inc\\ViewportInteraction\\OsgWidgetPickEngine\.h" />\r?\n',
		r'\s*<ClCompile Include="source\\ViewportInteraction\\OsgWidgetPickEngine\.cpp" />\r?\n',
	]
	for pat in patterns:
		text = re.sub(pat, "\n", text)
	for old, new in CONSUMER_INCLUDE_REPLACEMENTS:
		text = text.replace(old, new)
	# 确保能找到迁入 Host 的 Osg 头
	if "CloudSimHost/inc/osg" not in text.replace("\\", "/"):
		text = text.replace(
			"../../Host/CloudSimHost/inc;",
			"../../Host/CloudSimHost/inc;../../Host/CloudSimHost/inc/osg;../../Host/CloudSimHost/inc/pluginhost;",
		)
	if text == orig:
		print("NOCHANGE Widget.vcxproj")
		return
	if dry:
		print("WOULD_PATCH Widget.vcxproj")
		return
	path.write_text(text, encoding="utf-8", newline="\r\n")
	print("PATCHED Widget.vcxproj")


def patch_consumer_vcxprojs(dry: bool) -> None:
	for path in (ROOT / "src").rglob("*.vcxproj"):
		if path.parent == HOST:
			continue
		if path.name == "Widget.vcxproj":
			continue
		text = path.read_text(encoding="utf-8")
		if "CloudSimPluginHost" not in text and "CloudSimPluginHost" not in text.replace("/", "\\"):
			continue
		orig = text
		for old, new in CONSUMER_INCLUDE_REPLACEMENTS:
			text = text.replace(old, new)
		if text == orig:
			continue
		if dry:
			print(f"WOULD_PATCH {path.relative_to(ROOT)}")
			continue
		path.write_text(text, encoding="utf-8", newline="\r\n")
		print(f"PATCHED {path.relative_to(ROOT)}")


def remove_pluginhost_from_sln(dry: bool) -> None:
	sln = ROOT / "CloudSim.sln"
	text = sln.read_text(encoding="utf-8")
	# 匹配 Project 块
	pat = re.compile(
		r'Project\("[^"]+"\)\s*=\s*"CloudSimPluginHost".*?\nEndProject\r?\n',
		re.S,
	)
	new, n = pat.subn("", text)
	if n == 0:
		print("NOCHANGE CloudSim.sln (no CloudSimPluginHost project)")
		return
	# 清理残留的 ProjectConfiguration 行（按 GUID 较难；VS 可容忍孤立配置）
	if dry:
		print(f"WOULD_REMOVE CloudSimPluginHost from sln ({n} block)")
		return
	sln.write_text(new, encoding="utf-8", newline="\r\n")
	print(f"REMOVED CloudSimPluginHost from CloudSim.sln ({n})")


def relocate_pluginhost_docs(dry: bool) -> None:
	guide = PLUGIN / "DEVELOPER_GUIDE.md"
	if guide.exists():
		git_mv(guide, HOST / "DEVELOPER_GUIDE_PluginHost.md", dry)
	# 留下跳转说明，避免旧链接 404
	stub = (
		"# CloudSimPluginHost（已迁入 Host）\n\n"
		"源码与指南已迁至 "
		"[`src/Host/CloudSimHost/`](../../Host/CloudSimHost/) "
		"（`inc/pluginhost`、`source/pluginhost`；"
		"[DEVELOPER_GUIDE_PluginHost.md](../../Host/CloudSimHost/DEVELOPER_GUIDE_PluginHost.md)）。\n"
		"独立 `CloudSimPluginHost.vcxproj` 已从产品 sln 移除；产品路径编入 `CloudSimHost.dll`。\n"
	)
	stub_path = PLUGIN / "README.md"
	if dry:
		print(f"WOULD_WRITE {stub_path.relative_to(ROOT)}")
		return
	PLUGIN.mkdir(parents=True, exist_ok=True)
	stub_path.write_text(stub, encoding="utf-8", newline="\r\n")
	print(f"WROTE {stub_path.relative_to(ROOT)}")


def main() -> int:
	ap = argparse.ArgumentParser()
	ap.add_argument("--dry-run", action="store_true")
	args = ap.parse_args()
	dry = args.dry_run
	print("=== Widget → Host/osg ===")
	move_widget(dry)
	print("=== PluginHost → Host/pluginhost ===")
	move_pluginhost(dry)
	relocate_pluginhost_docs(dry)
	print("=== rewrite vcxproj ===")
	rewrite_vcxproj(HOST / "CloudSimHost.vcxproj", dry)
	rewrite_vcxproj(HOST / "CloudSimHostHeadless.vcxproj", dry)
	clean_widget_vcxproj(dry)
	patch_consumer_vcxprojs(dry)
	remove_pluginhost_from_sln(dry)
	return 0



if __name__ == "__main__":
	sys.exit(main())
