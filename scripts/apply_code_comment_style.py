#!/usr/bin/env python3
"""批量清理 AI 味注释（不改正文逻辑）。配合人工 Why 注释维护。"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"

REMOVE_LINE_PATTERNS = [
    re.compile(r"^\s*//\s*【中文】"),
    re.compile(r"^\s*//\s*【English】"),
    re.compile(r"^\s*///\s*【中文】"),
    re.compile(r"^\s*///\s*【English】"),
]

REWRITE_TRIPLE = [
    (re.compile(r"///\s*【中文】(.+?)【English】.+", re.DOTALL), r"/// \1"),
]

REWRITE_PHRASES = [
    (r"用于", "供"),
    (r"该方法", ""),
    (r"将被", ""),
    (r"。\s*$", ""),
]


def clean_file(path: Path) -> bool:
    text = path.read_text(encoding="utf-8", errors="replace")
    orig = text
    lines = text.splitlines(keepends=True)
    out: list[str] = []
    for line in lines:
        stripped = line.rstrip("\n\r")
        if any(p.match(stripped) for p in REMOVE_LINE_PATTERNS):
            continue
        for pat, repl in REWRITE_PHRASES:
            if stripped.strip().startswith(("//", "///")):
                stripped = re.sub(pat, repl, stripped)
        out.append(stripped + ("\n" if line.endswith("\n") else ""))
    text = "".join(out)
    if text != orig:
        path.write_text(text, encoding="utf-8")
        return True
    return False


def main() -> int:
    changed = 0
    for path in sorted(SRC.rglob("*.cpp")) + sorted(SRC.rglob("*.h")):
        if "pch" in path.name.lower():
            continue
        if clean_file(path):
            print(path.relative_to(ROOT))
            changed += 1
    print(f"done: {changed} files")
    return 0


if __name__ == "__main__":
    sys.exit(main())
