#!/usr/bin/env python3
"""Run clang-format -i on all product sources (cwd = CloudSim root for -style=file)."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _source_paths import ROOT, iter_sources  # noqa: E402

DEFAULT_CLANG_FORMAT = Path(
    r"C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Tools\Llvm\x64\bin\clang-format.exe"
)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--clang-format", type=Path, default=DEFAULT_CLANG_FORMAT)
    args = ap.parse_args()

    cf = args.clang_format
    if not cf.exists():
        print(f"clang-format not found: {cf}", file=sys.stderr)
        return 1

    files = [str(p) for p in iter_sources()]
    print(f"Formatting {len(files)} files with {cf}...")
    batch = 40
    failures = 0
    for i in range(0, len(files), batch):
        chunk = files[i : i + batch]
        cmd = [str(cf), "-i", "-style=file"] + chunk
        r = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True)
        if r.returncode != 0:
            failures += 1
            err = (r.stderr or r.stdout or "").strip()
            print(f"batch {i} rc={r.returncode}: {err[:200]}")
    print(f"Done. batch_failures={failures}")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
