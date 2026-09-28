# -*- coding: utf-8 -*-
"""校验插件 IID 与 SDK 头版本一致，防止 bump 后 moc 未重跑导致全拒载（#5）。

用法（CloudSim 根）:
  python scripts/check_plugin_iid_alignment.py
  python scripts/check_plugin_iid_alignment.py --skip-bin   # 只比对头文件双真源
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPO_BIN = ROOT.parent / "bin"

PLUGIN_GLOBAL = ROOT / "src/Plugins/CloudSimPluginSDK/inc/cloudsim_plugin_sdk_global.h"
PLUGIN_IID_H = ROOT / "src/Plugins/CloudSimPluginSDK/inc/ICloudSimPlugin.h"
AI_GLOBAL = ROOT / "src/Plugins/CloudSimAiSDK/inc/cloudsim_ai_sdk_global.h"
AI_IID_H = ROOT / "src/Plugins/CloudSimAiSDK/inc/ICloudSimAiPlugin.h"

VERSION_STR_RE = re.compile(
	r'#\s*define\s+CLOUDSIM_PLUGIN_SDK_VERSION_STR\s+"([^"]+)"'
)
AI_VERSION_STR_RE = re.compile(
	r'#\s*define\s+CLOUDSIM_AI_SDK_VERSION_STR\s+"([^"]+)"'
)
PLUGIN_IID_RE = re.compile(
	r'#\s*define\s+CloudSimPlugin_iid\s+"com\.cloudsim\.ICloudSimPlugin/1\.0\.([^"]+)"'
)
AI_IID_RE = re.compile(
	r'#\s*define\s+CloudSimAiPlugin_iid\s+"com\.cloudsim\.ICloudSimAiPlugin/1\.0\.([^"]+)"'
)
# 旧无版本后缀亦识别，便于报错
AI_IID_LEGACY_RE = re.compile(
	r'#\s*define\s+CloudSimAiPlugin_iid\s+"com\.cloudsim\.ICloudSimAiPlugin/1\.0"'
)

EMBEDDED_PLUGIN_IID_RE = re.compile(
	rb"com\.cloudsim\.ICloudSimPlugin/1\.0\.(0x[0-9A-Fa-f]+)"
)


def read_text(path: Path) -> str:
	return path.read_text(encoding="utf-8", errors="replace")


def check_headers() -> list[str]:
	errors: list[str] = []
	pg = read_text(PLUGIN_GLOBAL)
	pi = read_text(PLUGIN_IID_H)
	m_str = VERSION_STR_RE.search(pg)
	m_iid = PLUGIN_IID_RE.search(pi)
	if not m_str:
		errors.append(f"missing CLOUDSIM_PLUGIN_SDK_VERSION_STR in {PLUGIN_GLOBAL}")
	if not m_iid:
		errors.append(f"missing versioned CloudSimPlugin_iid literal in {PLUGIN_IID_H}")
	if m_str and m_iid and m_str.group(1) != m_iid.group(1):
		errors.append(
			f"Plugin IID dual-source drift: VERSION_STR={m_str.group(1)} "
			f"vs CloudSimPlugin_iid suffix={m_iid.group(1)} "
			f"(bump 时须同步改两处，并 Rebuild 全部插件 / 清 moc)"
		)

	ag = read_text(AI_GLOBAL)
	ai = read_text(AI_IID_H)
	a_str = AI_VERSION_STR_RE.search(ag)
	a_iid = AI_IID_RE.search(ai)
	if AI_IID_LEGACY_RE.search(ai) and not a_iid:
		errors.append(
			f"CloudSimAiPlugin_iid 缺少版本后缀（须 com.cloudsim.ICloudSimAiPlugin/1.0.<VERSION_STR>）"
		)
	if not a_str:
		errors.append(f"missing CLOUDSIM_AI_SDK_VERSION_STR in {AI_GLOBAL}")
	if a_str and a_iid and a_str.group(1) != a_iid.group(1):
		errors.append(
			f"Ai IID dual-source drift: VERSION_STR={a_str.group(1)} "
			f"vs CloudSimAiPlugin_iid suffix={a_iid.group(1)}"
		)
	return errors


def expected_plugin_suffix() -> str | None:
	m = VERSION_STR_RE.search(read_text(PLUGIN_GLOBAL))
	return m.group(1) if m else None


def scan_plugin_dlls(expected_suffix: str) -> list[str]:
	errors: list[str] = []
	checked = 0
	for cfg_dir, label in ((REPO_BIN / "x64d" / "plugins", "Debug"), (REPO_BIN / "x64" / "plugins", "Release")):
		if not cfg_dir.is_dir():
			continue
		for dll in sorted(cfg_dir.rglob("*.dll")):
			# 跳过非 CloudSim 插件命名空间旁路拷贝
			if "com.cloudsim." not in dll.parent.name and "com.cloudsim." not in dll.name.lower():
				# 插件目录名通常为 com.cloudsim.*
				if not dll.parent.name.startswith("com."):
					continue
			data = dll.read_bytes()
			found = set(m.group(1).decode("ascii") for m in EMBEDDED_PLUGIN_IID_RE.finditer(data))
			if not found:
				errors.append(f"{label}: no embedded Plugin IID in {dll.relative_to(REPO_BIN)}")
				continue
			checked += 1
			if expected_suffix not in found:
				errors.append(
					f"{label}: {dll.relative_to(REPO_BIN)} has IID suffix(es) {sorted(found)}, "
					f"expected {expected_suffix} — 清 moc 后 Rebuild 该插件"
				)
			elif len(found) > 1:
				errors.append(
					f"{label}: {dll.relative_to(REPO_BIN)} embeds mixed IIDs {sorted(found)}"
				)
	if checked == 0:
		print("SKIP bin scan: no plugin DLLs under bin/x64d|x64/plugins (headers-only check)")
	else:
		print(f"OK bin scan: {checked} plugin DLL(s) checked against {expected_suffix}")
	return errors


def main() -> int:
	ap = argparse.ArgumentParser()
	ap.add_argument("--skip-bin", action="store_true")
	args = ap.parse_args()

	errors = check_headers()
	if not errors:
		print("OK headers: Plugin/Ai IID literals match VERSION_STR")

	if not args.skip_bin:
		suffix = expected_plugin_suffix()
		if suffix:
			errors.extend(scan_plugin_dlls(suffix))

	if errors:
		for e in errors:
			print(f"FAIL: {e}", file=sys.stderr)
		return 1
	print("OK plugin IID alignment")
	return 0


if __name__ == "__main__":
	sys.exit(main())
