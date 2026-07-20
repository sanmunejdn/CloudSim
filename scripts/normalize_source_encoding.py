#!/usr/bin/env python3
"""Rewrite product sources as UTF-8 with BOM + CRLF. Run after clang-format."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _source_paths import iter_sources  # noqa: E402

BOM = b"\xef\xbb\xbf"


def normalize_text(raw: bytes) -> bytes:
    if raw.startswith(BOM):
        text = raw[len(BOM) :].decode("utf-8")
    else:
        # Prefer utf-8; fall back to gbk for legacy files still on disk as GBK
        try:
            text = raw.decode("utf-8")
        except UnicodeDecodeError:
            text = raw.decode("gbk")
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    text = text.replace("\n", "\r\n")
    return BOM + text.encode("utf-8")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    changed = 0
    for path in iter_sources():
        raw = path.read_bytes()
        out = normalize_text(raw)
        if out == raw:
            continue
        changed += 1
        rel = path
        print(f"{'DRY ' if args.dry_run else ''}BOM+CRLF: {rel}")
        if not args.dry_run:
            path.write_bytes(out)
    print(f"Done. changed={changed}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
