#!/usr/bin/env python3
"""兼容入口：应用 Logo 已改为 SVG 源稿，请运行 generate_logo_from_svg.ps1。

历史 pillow 几何（云+坐标轴）已废弃；真源：
  CloudSim/src/UI/CloudSimUiAssets/resources/logo/cloudsim_logo.svg
  （设计全稿含 glow：cloudsim_logo_full.svg）
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> int:
    ps1 = Path(__file__).resolve().parent / "generate_logo_from_svg.ps1"
    if not ps1.is_file():
        print(f"missing {ps1}", file=sys.stderr)
        return 1
    cmd = [
        "powershell",
        "-ExecutionPolicy",
        "Bypass",
        "-File",
        str(ps1),
    ]
    print("delegating to", ps1.name)
    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
