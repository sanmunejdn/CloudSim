#!/usr/bin/env python3
"""Ensure /// @file / /// @brief exist when missing (after include guard)."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _source_paths import ROOT, iter_sources, project_name_for  # noqa: E402


def read_text(path: Path) -> str:
    raw = path.read_bytes()
    if raw.startswith(b"\xef\xbb\xbf"):
        return raw[3:].decode("utf-8", errors="replace")
    try:
        return raw.decode("utf-8")
    except UnicodeDecodeError:
        return raw.decode("gbk", errors="replace")


def write_text(path: Path, text: str) -> None:
    data = text.replace("\r\n", "\n").replace("\r", "\n").encode("utf-8")
    path.write_bytes(data)


def brief_for(path: Path, text: str) -> str:
    for m in re.finditer(r"^///\s+(.+)$", text, re.MULTILINE):
        line = m.group(1).strip()
        if line.startswith("@"):
            continue
        if not line:
            continue
        if "\ufffd" in line:
            continue
        return line
    if path.name.endswith("_global.h"):
        return f"{project_name_for(path)} 导出宏"
    if path.stem.lower() == "pch":
        return "预编译头"
    if path.suffix.lower() in {".h", ".hpp", ".hh", ".hxx"}:
        return f"{path.stem} 接口"
    return f"{path.stem} 实现"


def has_file_header(text: str) -> bool:
    head = "\n".join(text.splitlines()[:50])
    return "@file" in head or bool(re.search(r"^///\s*@brief\b", head, re.MULTILINE))


def insert_file_header(text: str, path: Path) -> str:
    brief = brief_for(path, text)
    block = f"/// @file {path.name}\n/// @brief {brief}\n\n"
    m = re.match(r"(#ifndef\s+\w+\s*\n#define\s+\w+\s*\n)", text)
    if m:
        rest = text[m.end() :]
        # Avoid double blank
        rest = rest.lstrip("\n")
        return m.group(1) + "\n" + block + rest
    return block + text.lstrip("\n")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    changed = 0
    for path in iter_sources():
        text = read_text(path).replace("\r\n", "\n")
        if has_file_header(text):
            continue
        new = insert_file_header(text, path)
        if new == text:
            continue
        changed += 1
        print(f"{'DRY ' if args.dry_run else ''}HEADER: {path.relative_to(ROOT)}")
        if not args.dry_run:
            write_text(path, new)
    print(f"Done. changed={changed}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
