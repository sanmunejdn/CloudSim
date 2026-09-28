"""git post-commit hook 调用：把最新 commit 上报给 BugTracker 联动。

用法（在目标仓库根目录）：
    python scan_commits.py [--api http://127.0.0.1:8500] [--token <token>]

环境变量：BT_API / BT_INTEGRATION_TOKEN 可替代参数。
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import urllib.request


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--api", default=os.environ.get("BT_API",
                                                        "http://127.0.0.1:8500"))
    parser.add_argument("--token",
                        default=os.environ.get("BT_INTEGRATION_TOKEN", ""))
    parser.add_argument("--rev", default="HEAD", help="要扫描的 commit")
    args = parser.parse_args()

    if not args.token:
        print("[bugtracker] 未配置 token（--token 或 BT_INTEGRATION_TOKEN），跳过",
              file=sys.stderr)
        return 0  # hook 永远不阻断提交

    try:
        # git 输出为 UTF-8，须显式指定，避免 Windows 默认 GBK 解码失败
        message = subprocess.check_output(
            ["git", "log", "-1", "--pretty=%B", args.rev],
            encoding="utf-8", errors="replace").strip()
        commit_hash = subprocess.check_output(
            ["git", "rev-parse", args.rev],
            encoding="utf-8", errors="replace").strip()
        author = subprocess.check_output(
            ["git", "log", "-1", "--pretty=%an", args.rev],
            encoding="utf-8", errors="replace").strip()
        repo = os.path.basename(os.getcwd())
    except subprocess.CalledProcessError as exc:
        print(f"[bugtracker] git 读取失败: {exc}", file=sys.stderr)
        return 0

    if "#" not in message:
        return 0  # 没有 bug 引用，直接跳过

    payload = json.dumps({"repo": repo, "hash": commit_hash,
                          "message": message, "author": author}).encode()
    request = urllib.request.Request(
        f"{args.api}/api/integrations/commit", data=payload,
        headers={"Content-Type": "application/json",
                 "X-Integration-Token": args.token})
    try:
        with urllib.request.urlopen(request, timeout=10) as resp:
            result = json.loads(resp.read())
        print(f"[bugtracker] commit 联动: {result}")
    except Exception as exc:  # 网络/服务不可用不阻断提交
        print(f"[bugtracker] 上报失败（不影响提交）: {exc}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
