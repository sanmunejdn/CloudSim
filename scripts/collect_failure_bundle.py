"""打包失败产物：日志 / junit / crash dump / 可选场景 JSON"""

from __future__ import annotations

import argparse
import shutil
import zipfile
from datetime import datetime, timezone
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cloudsim-root", default="", help="CloudSim/ 根；默认脚本上两级")
    ap.add_argument("--bin-dir", default="", help="含 crash/ 的 bin 目录")
    ap.add_argument("--extra", action="append", default=[], help="额外文件或目录")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    root = Path(args.cloudsim_root) if args.cloudsim_root else Path(__file__).resolve().parents[1]
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out = Path(args.out) if args.out else root / "artifacts" / "failures" / f"bundle_{stamp}.zip"
    out.parent.mkdir(parents=True, exist_ok=True)

    staging = out.parent / f"_staging_{stamp}"
    if staging.exists():
        shutil.rmtree(staging)
    staging.mkdir(parents=True)

    checks = root / "artifacts" / "checks"
    if checks.is_dir():
        shutil.copytree(checks, staging / "checks", dirs_exist_ok=True)

    soak = root / "artifacts" / "soak"
    if soak.is_dir():
        shutil.copytree(soak, staging / "soak", dirs_exist_ok=True)

    if args.bin_dir:
        crash = Path(args.bin_dir) / "crash"
        if crash.is_dir():
            shutil.copytree(crash, staging / "crash", dirs_exist_ok=True)

    for e in args.extra:
        p = Path(e)
        if p.is_file():
            shutil.copy2(p, staging / p.name)
        elif p.is_dir():
            shutil.copytree(p, staging / p.name, dirs_exist_ok=True)

    with zipfile.ZipFile(out, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for f in staging.rglob("*"):
            if f.is_file():
                zf.write(f, f.relative_to(staging).as_posix())

    shutil.rmtree(staging, ignore_errors=True)
    print(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
