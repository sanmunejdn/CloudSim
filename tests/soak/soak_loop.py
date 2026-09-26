"""稳定性循环：建模/碰撞设置/侧车往返，记录内存与耗时"""

from __future__ import annotations

import argparse
import csv
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import psutil
import requests

REPO_ROOT = Path(__file__).resolve().parents[2]


def bin_dir(configuration: str) -> Path:
    return REPO_ROOT.parent / ("bin/x64d" if configuration.lower() == "debug" else "bin/x64")


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def wait_ready(base: str, proc: subprocess.Popen, timeout: float = 60.0) -> None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if proc.poll() is not None:
            raise RuntimeError(f"CloudSimWeb exited early: {proc.returncode}")
        try:
            if requests.get(f"{base}/api/help", timeout=2).status_code == 200:
                return
        except requests.RequestException:
            pass
        time.sleep(0.5)
    raise RuntimeError("CloudSimWeb not ready")


def one_round(base: str) -> None:
    r = requests.post(
        f"{base}/api/geomodeling/op",
        json={"op": "primitive", "kind": "box", "lengthMm": 40, "widthMm": 30, "heightMm": 20},
        timeout=60,
    )
    r.raise_for_status()
    body = r.json()
    if not body.get("ok"):
        raise RuntimeError(body)
    bid = body["backendId"]
    summary = requests.get(f"{base}/api/geomodeling/summary", timeout=30).json()
    feats = next(b["features"] for b in summary["bodies"] if b["backendId"] == bid)
    pad = next(f for f in feats if f.get("kind") == "Pad")
    requests.post(
        f"{base}/api/geomodeling/op",
        json={"op": "patch", "backendId": bid, "featureId": pad["id"], "lengthMm": 25.0},
        timeout=60,
    ).raise_for_status()
    requests.post(f"{base}/api/geomodeling/op", json={"op": "undo"}, timeout=30).raise_for_status()
    requests.post(f"{base}/api/geomodeling/op", json={"op": "redo"}, timeout=30).raise_for_status()

    cs = requests.get(f"{base}/api/robot/collision-settings", timeout=15).json()
    margin = float(cs.get("securityMarginMm", 1.0))
    payload = {k: v for k, v in cs.items() if k != "ok"}
    payload["securityMarginMm"] = margin
    requests.put(f"{base}/api/robot/collision-settings", json=payload, timeout=15).raise_for_status()

    requests.put(
        f"{base}/api/sidecar/geometricModeling",
        json={"soak": True, "t": time.time()},
        timeout=15,
    ).raise_for_status()


def slope(xs: list[float], ys: list[float]) -> float:
    n = len(xs)
    if n < 2:
        return 0.0
    mx = sum(xs) / n
    my = sum(ys) / n
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    den = sum((x - mx) ** 2 for x in xs)
    return num / den if den else 0.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--configuration", default=os.environ.get("CLOUDSIM_TEST_CONFIG", "Debug"))
    ap.add_argument("--rounds", type=int, default=200)
    ap.add_argument("--max-mb-per-round", type=float, default=0.5, help="WorkingSet 线性斜率上限 MB/轮")
    ap.add_argument("--out", default="", help="CSV 输出路径")
    args = ap.parse_args()

    bdir = bin_dir(args.configuration)
    exe = bdir / "CloudSimWeb.exe"
    if not exe.is_file():
        print(f"missing {exe}", file=sys.stderr)
        return 2

    out_dir = REPO_ROOT / "artifacts" / "soak"
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = Path(args.out) if args.out else out_dir / f"soak_{args.configuration}_{int(time.time())}.csv"

    env = os.environ.copy()
    qt_bin = Path(os.environ.get("QTDIR", r"D:\Qt\Qt5.14.2\5.14.2\msvc2017_64")) / "bin"
    osg_bin = REPO_ROOT.parent / "bin" / "SDK" / "OSG3.6.5" / "bin"
    path_prefix = os.pathsep.join(str(p) for p in (bdir, qt_bin, osg_bin) if Path(p).exists())
    env["PATH"] = path_prefix + os.pathsep + env.get("PATH", "")

    port = free_port()
    proc = subprocess.Popen(
        [str(exe), f"--port={port}"],
        cwd=str(bdir),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        env=env,
        creationflags=subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0,
    )
    base = f"http://127.0.0.1:{port}"
    try:
        wait_ready(base, proc)
        ps = psutil.Process(proc.pid)
        rows: list[tuple[int, float, float]] = []
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["round", "elapsed_sec", "rss_mb"])
            for i in range(1, args.rounds + 1):
                t0 = time.perf_counter()
                one_round(base)
                elapsed = time.perf_counter() - t0
                rss_mb = ps.memory_info().rss / (1024 * 1024)
                rows.append((i, elapsed, rss_mb))
                w.writerow([i, f"{elapsed:.4f}", f"{rss_mb:.2f}"])
                if i % 20 == 0:
                    print(f"round {i}/{args.rounds} rss={rss_mb:.1f}MB elapsed={elapsed:.3f}s")

        xs = [float(r[0]) for r in rows]
        ys = [r[2] for r in rows]
        s = slope(xs, ys)
        print(f"CSV: {csv_path}")
        print(f"memory slope: {s:.4f} MB/round (limit {args.max_mb_per_round})")
        if s > args.max_mb_per_round:
            print("FAIL: memory growth exceeds limit", file=sys.stderr)
            return 1
        print("PASS")
        return 0
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()


if __name__ == "__main__":
    raise SystemExit(main())
