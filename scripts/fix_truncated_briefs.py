#!/usr/bin/env python3
"""Rewrite /// @brief lines without 48-char truncation."""

from __future__ import annotations

import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _source_paths import ROOT, iter_sources, project_name_for  # noqa: E402


def read_text(path: Path) -> str:
    raw = path.read_bytes()
    if raw.startswith(b"\xef\xbb\xbf"):
        return raw[3:].decode("utf-8", errors="replace")
    return raw.decode("utf-8", errors="replace")


def brief_for(path: Path, text: str) -> str:
    for m in re.finditer(r"^///\s+(.+)$", text, re.MULTILINE):
        line = m.group(1).strip()
        if line.startswith("@"):
            continue
        if not line or "\ufffd" in line:
            continue
        # Prefer a later class/API doc over a truncated @brief already in file
        if line.startswith("@brief"):
            continue
        return line
    if path.name.endswith("_global.h"):
        return f"{project_name_for(path)} 导出宏"
    if path.stem.lower() == "pch":
        return "预编译头"
    if path.suffix.lower() in {".h", ".hpp", ".hh", ".hxx"}:
        return f"{path.stem} 接口"
    return f"{path.stem} 实现"


def main() -> int:
    changed = 0
    for path in iter_sources():
        text = read_text(path).replace("\r\n", "\n")
        m = re.search(r"^/// @brief (.+)$", text, re.MULTILINE)
        if not m:
            continue
        new_brief = brief_for(path, text)
        old_brief = m.group(1)
        if new_brief == old_brief:
            continue
        # Only expand truncated / replace if new is longer or old looks cut
        if len(new_brief) <= len(old_brief) and not (
            len(old_brief) >= 40 and old_brief[-1] not in "。）)接口实现宏头"
        ):
            continue
        text2 = text[: m.start(1)] + new_brief + text[m.end(1) :]
        path.write_bytes(text2.encode("utf-8"))
        changed += 1
        print(f"BRIEF: {path.relative_to(ROOT)}")
    print(f"Done. changed={changed}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
