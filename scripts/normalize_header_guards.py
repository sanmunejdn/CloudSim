#!/usr/bin/env python3
"""Normalize header include guards to PROJECT_STEM_H; remove #pragma once."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _source_paths import guard_macro, iter_sources  # noqa: E402

HEADER_SUFFIXES = {".h", ".hpp", ".hh", ".hxx"}


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


def normalize_newlines(text: str) -> str:
    return text.replace("\r\n", "\n").replace("\r", "\n")


def find_outer_guard(text: str) -> tuple[int, int, int, int, str] | None:
    """Return (ifndef_start, define_end, endif_start, endif_end, macro) for outer include guard."""
    m_if = re.search(r"(?m)^#\s*ifndef\s+(\w+)\s*$", text)
    if not m_if:
        m_if = re.search(r"(?m)^#\s*if\s+!\s*defined\s*\(\s*(\w+)\s*\)\s*$", text)
    if not m_if:
        return None
    macro = m_if.group(1)
    # define must follow closely (allow blank lines)
    after = text[m_if.end() :]
    m_def = re.match(r"\s*#\s*define\s+" + re.escape(macro) + r"\s*\n", after)
    if not m_def:
        return None
    define_end = m_if.end() + m_def.end()

    # Matching outer #endif: last #endif in file that is at column 0
    endifs = list(re.finditer(r"(?m)^#\s*endif\b[^\n]*\n?", text))
    if not endifs:
        return None
    m_end = endifs[-1]
    return m_if.start(), define_end, m_end.start(), m_end.end(), macro


def strip_pragma_once(text: str) -> str:
    return re.sub(r"(?mi)^#\s*pragma\s+once\s*\n+", "", text, count=1)


def process(path: Path) -> str | None:
    macro = guard_macro(path)
    text = normalize_newlines(read_text(path))
    text = strip_pragma_once(text)

    g = find_outer_guard(text)
    if g:
        if_start, define_end, endif_start, endif_end, _old = g
        body = text[define_end:endif_start]
        # Keep any content before the outer ifndef (should be rare)
        prefix = text[:if_start].rstrip() + "\n" if text[:if_start].strip() else ""
    else:
        prefix = ""
        body = text

    body = body.strip("\n") + "\n"
    new = f"{prefix}#ifndef {macro}\n#define {macro}\n\n{body}\n#endif // {macro}\n"
    old_norm = normalize_newlines(read_text(path))
    if new == old_norm:
        return None
    # Already correct macro and structure
    if (
        f"#ifndef {macro}\n#define {macro}\n" in old_norm
        and f"#endif // {macro}" in old_norm
        and "#pragma once" not in old_norm.lower()
    ):
        # May still differ in whitespace — only rewrite if macro wrong or pragma present
        if find_outer_guard(old_norm) and find_outer_guard(old_norm)[4] == macro:
            return None
    return new


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    changed = 0
    for path in iter_sources():
        if path.suffix.lower() not in HEADER_SUFFIXES:
            continue
        new = process(path)
        if new is None:
            continue
        changed += 1
        from _source_paths import ROOT

        print(f"{'DRY ' if args.dry_run else ''}GUARD {guard_macro(path)}: {path.relative_to(ROOT)}")
        if not args.dry_run:
            write_text(path, new)
    print(f"Done. changed={changed}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
