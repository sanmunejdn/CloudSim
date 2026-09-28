"""CI 失败 → BugTracker 草稿：解析 artifacts/checks/<stamp>/ 下的 junit XML。

用法（在 CloudSim 仓库根目录）：
    python tools/bugtracker/scripts/ci_failure_to_bug.py --stamp 20260928-101500
    python tools/bugtracker/scripts/ci_failure_to_bug.py   # 自动取最新一批

环境变量：BT_API / BT_INTEGRATION_TOKEN（同 scan_commits.py）。
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
import urllib.request
import xml.etree.ElementTree as ET


def find_latest_stamp(checks_dir: str) -> str:
    stamps = [d for d in os.listdir(checks_dir)
              if os.path.isdir(os.path.join(checks_dir, d))]
    if not stamps:
        raise SystemExit(f"没有可用的 checks 批次: {checks_dir}")
    return sorted(stamps)[-1]


def parse_junit(dir_path: str) -> tuple[int, int, list[dict]]:
    """返回 (总用例数, 失败数, 失败明细)。"""
    total = failed = 0
    failures: list[dict] = []
    for path in glob.glob(os.path.join(dir_path, "*junit*.xml")):
        try:
            tree = ET.parse(path)
        except ET.ParseError:
            continue
        for case in tree.iter("testcase"):
            total += 1
            name = f"{case.get('classname', '')}::{case.get('name', '?')}"
            for tag in ("failure", "error"):
                node = case.find(tag)
                if node is not None:
                    failed += 1
                    failures.append({
                        "name": name,
                        "message": (node.get("message")
                                    or (node.text or ""))[:500],
                    })
                    break
    return total, failed, failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stamp", default="", help="checks 批次目录名；缺省取最新")
    parser.add_argument("--checks-dir", default="artifacts/checks")
    parser.add_argument("--project-key", default="CLOUDSIM")
    parser.add_argument("--api", default=os.environ.get("BT_API",
                                                        "http://127.0.0.1:8500"))
    parser.add_argument("--token",
                        default=os.environ.get("BT_INTEGRATION_TOKEN", ""))
    args = parser.parse_args()

    if not args.token:
        print("[bugtracker] 未配置 BT_INTEGRATION_TOKEN", file=sys.stderr)
        return 2

    stamp = args.stamp or find_latest_stamp(args.checks_dir)
    dir_path = os.path.join(args.checks_dir, stamp)
    total, failed, failures = parse_junit(dir_path)
    if failed == 0:
        print(f"[bugtracker] 批次 {stamp} 无失败用例，无需提单")
        return 0

    payload = json.dumps({
        "stamp": stamp,
        "summary": f"CI 批次 {stamp}：{failed}/{total} 用例失败",
        "project_key": args.project_key,
        "failures": failures,
    }).encode()
    request = urllib.request.Request(
        f"{args.api}/api/integrations/ci-failure", data=payload,
        headers={"Content-Type": "application/json",
                 "X-Integration-Token": args.token})
    with urllib.request.urlopen(request, timeout=15) as resp:
        result = json.loads(resp.read())
    print(f"[bugtracker] 已生成 bug 草稿: #{result['bug_id']} "
          f"（项目 {result['project_key']}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
