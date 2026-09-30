# -*- coding: utf-8 -*-
"""Web fallback 护栏：部署目录若为 fallback 布局且落后于 React 源码则失败。

_archive/public-fallback 作行为对照，允许落后于 src（正式产物走 Vite）。
正式打包禁 CLOUDSIM_WEB_FALLBACK 见 Setup/packaging。

用法（CloudSim 根）:
  python scripts/check_web_fallback_freshness.py
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
UI_ROOT = ROOT / "web" / "cloudsim-web-ui"
ARCHIVE_ROOT = UI_ROOT / "_archive" / "public-fallback"
SRC_ROOT = UI_ROOT / "src"
PACKAGE_JSON = UI_ROOT / "package.json"
BIN_WEB_CANDIDATES = (
	ROOT.parent / "bin" / "x64d" / "web",
	ROOT.parent / "bin" / "x64" / "web",
)


def latest_mtime(root: Path) -> float | None:
	if not root.exists():
		return None
	if root.is_file():
		return root.stat().st_mtime
	latest = None
	for p in root.rglob("*"):
		if p.is_file():
			t = p.stat().st_mtime
			if latest is None or t > latest:
				latest = t
	return latest


def is_fallback_web_root(web: Path) -> bool:
	"""无 assets/*.js 且有根级 app.js → 疑似 public-fallback 拷贝"""
	if not web.is_dir():
		return False
	assets = web / "assets"
	has_vite_js = bool(list(assets.glob("*.js"))) if assets.is_dir() else False
	has_app_js = (web / "app.js").is_file()
	return has_app_js and not has_vite_js


def main() -> int:
	if not SRC_ROOT.is_dir():
		print(f"FAIL: missing src dir: {SRC_ROOT}", file=sys.stderr)
		return 1

	src_latest = latest_mtime(SRC_ROOT) or 0.0
	if PACKAGE_JSON.is_file():
		src_latest = max(src_latest, PACKAGE_JSON.stat().st_mtime)

	# 归档对照：仅提示，不阻断（React 为默认交付）
	if ARCHIVE_ROOT.is_dir():
		archive_latest = latest_mtime(ARCHIVE_ROOT)
		if archive_latest is not None and archive_latest < src_latest:
			print(
				f"NOTE: _archive/public-fallback older than src "
				f"(archive={archive_latest:.0f} src={src_latest:.0f}); "
				f"OK if packaging uses Vite assets only"
			)

	errors: list[str] = []
	for web in BIN_WEB_CANDIDATES:
		if not is_fallback_web_root(web):
			continue
		web_latest = latest_mtime(web)
		if web_latest is None:
			continue
		if web_latest < src_latest:
			errors.append(
				f"deployed fallback web stale: {web} "
				f"(web={web_latest:.0f} < src={src_latest:.0f}); "
				f"run Vite build or sync; do not ship CLOUDSIM_WEB_FALLBACK"
			)

	if errors:
		for e in errors:
			print(f"FAIL: {e}", file=sys.stderr)
		return 1

	print("OK web fallback freshness")
	return 0


if __name__ == "__main__":
	sys.exit(main())
