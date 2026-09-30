# -*- coding: utf-8 -*-
"""Host/Headless DLL 导出面护栏：基线符号缺失则失败；导出数异常膨胀告警。

用法（CloudSim 根）:
  python scripts/check_host_export_surface.py
  python scripts/check_host_export_surface.py --config release
  python scripts/check_host_export_surface.py --dll path\\to\\CloudSimHost.dll

不删 /WHOLEARCHIVE；本脚本只观测导出面，防止 Widget/Bootstrap 必需簇静默丢失。
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPO_BIN = ROOT.parent / "bin"

# Widget / Bootstrap / 插件 Host 契约必需名片段（mangled 导出名子串即可）
REQUIRED_FRAGMENTS_BOTH = (
	"DocumentHost@host@cloudsim",
	"PluginManager@",
	"PluginHostContext@",
	"?createDocumentHost@host@cloudsim@@",
	"?createHeadlessDocumentHost@host@cloudsim@@",
)

# 桌面 Host 额外（Headless 可不强制）
REQUIRED_FRAGMENTS_DESKTOP = (
	"?sceneOps@DocumentHost@host@cloudsim@@",
	"?pickObserve@DocumentHost@host@cloudsim@@",
)

# 软告警：导出条目超过该阈值打 WARN（不失败）
SOFT_EXPORT_WARN_THRESHOLD = 2500

EXPORT_LINE_RE = re.compile(
	r"^\s+\d+\s+[0-9A-Fa-f]+\s+[0-9A-Fa-f]+\s+(\S+)"
)


def find_dumpbin() -> Path | None:
	env = os.environ.get("DUMPBIN")
	if env and Path(env).is_file():
		return Path(env)
	vswhere = Path(os.environ.get("ProgramFiles(x86)", r"C:\Program Files (x86)")) / (
		"Microsoft Visual Studio/Installer/vswhere.exe"
	)
	if not vswhere.is_file():
		return None
	try:
		out = subprocess.check_output(
			[
				str(vswhere),
				"-latest",
				"-products",
				"*",
				"-requires",
				"Microsoft.VisualStudio.Component.VC.Tools.x86.x64",
				"-property",
				"installationPath",
			],
			text=True,
			stderr=subprocess.DEVNULL,
		).strip()
	except (subprocess.CalledProcessError, OSError):
		return None
	if not out:
		return None
	msvc = Path(out) / "VC" / "Tools" / "MSVC"
	if not msvc.is_dir():
		return None
	cands = sorted(msvc.glob("*/bin/HostX64/x64/dumpbin.exe"), reverse=True)
	return cands[0] if cands else None


def parse_exports(dumpbin: Path, dll: Path) -> list[str]:
	proc = subprocess.run(
		[str(dumpbin), "/EXPORTS", str(dll)],
		capture_output=True,
		text=True,
		encoding="utf-8",
		errors="replace",
	)
	if proc.returncode != 0:
		raise RuntimeError(
			f"dumpbin failed ({proc.returncode}) for {dll}:\n{proc.stderr or proc.stdout}"
		)
	names: list[str] = []
	for line in (proc.stdout or "").splitlines():
		m = EXPORT_LINE_RE.match(line)
		if m:
			names.append(m.group(1))
	return names


def check_dll(
	dumpbin: Path,
	dll: Path,
	required: tuple[str, ...],
	*,
	label: str,
) -> list[str]:
	errors: list[str] = []
	if not dll.is_file():
		errors.append(f"{label}: missing DLL {dll}")
		return errors
	try:
		exports = parse_exports(dumpbin, dll)
	except RuntimeError as e:
		errors.append(str(e))
		return errors
	blob = "\n".join(exports)
	for frag in required:
		if frag not in blob:
			errors.append(f"{label}: missing required export fragment {frag!r} in {dll.name}")
	n = len(exports)
	print(f"  {label}: {dll.name} exports={n}")
	if n > SOFT_EXPORT_WARN_THRESHOLD:
		print(
			f"  WARN {label}: export count {n} > soft threshold {SOFT_EXPORT_WARN_THRESHOLD} "
			f"(WHOLEARCHIVE 纳入可接受；请人工确认是否意外膨胀)"
		)
	return errors


def bin_dir(config: str) -> Path:
	cfg = config.lower()
	if cfg in ("debug", "x64d"):
		return REPO_BIN / "x64d"
	if cfg in ("release", "x64"):
		return REPO_BIN / "x64"
	raise SystemExit(f"unknown --config {config!r} (use debug|release)")


def main() -> int:
	ap = argparse.ArgumentParser(description="Host/Headless export-surface allowlist guard")
	ap.add_argument(
		"--config",
		default="debug",
		help="debug→bin/x64d, release→bin/x64（默认 debug）",
	)
	ap.add_argument("--dll", action="append", default=[], help="显式 DLL 路径（可多次）")
	ap.add_argument(
		"--skip-missing",
		action="store_true",
		help="DLL 不存在时跳过（不失败）；用于未编过的工作树",
	)
	args = ap.parse_args()

	dumpbin = find_dumpbin()
	if not dumpbin:
		print("ERROR: dumpbin.exe not found (need VS VC tools or set DUMPBIN)", file=sys.stderr)
		return 2

	errors: list[str] = []
	if args.dll:
		for p in args.dll:
			dll = Path(p)
			label = dll.stem
			req = REQUIRED_FRAGMENTS_BOTH
			if "Headless" not in dll.name:
				req = REQUIRED_FRAGMENTS_BOTH + REQUIRED_FRAGMENTS_DESKTOP
			if args.skip_missing and not dll.is_file():
				print(f"  skip missing {dll}")
				continue
			errors.extend(check_dll(dumpbin, dll, req, label=label))
	else:
		b = bin_dir(args.config)
		host = b / "CloudSimHost.dll"
		headless = b / "CloudSimHostHeadless.dll"
		print(f"dumpbin={dumpbin}")
		print(f"bin={b}")
		for dll, label, req in (
			(host, "CloudSimHost", REQUIRED_FRAGMENTS_BOTH + REQUIRED_FRAGMENTS_DESKTOP),
			(headless, "CloudSimHostHeadless", REQUIRED_FRAGMENTS_BOTH),
		):
			if args.skip_missing and not dll.is_file():
				print(f"  skip missing {dll}")
				continue
			errors.extend(check_dll(dumpbin, dll, req, label=label))

	for e in errors:
		print(f"ERROR: {e}", file=sys.stderr)
	if errors:
		return 1
	print("OK host export surface")
	return 0


if __name__ == "__main__":
	sys.exit(main())
