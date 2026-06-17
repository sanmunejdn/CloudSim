#!/usr/bin/env python3
"""修复 CloudSim 旧版 writeNpyInt64 导出的标签 .npy（magic 仅 4 字节）。"""

from __future__ import annotations

import argparse
import os
import re
import struct
import sys

import numpy as np
import numpy.lib.format as npy_format


def _is_legacy_broken_npy(path: str) -> bool:
    with open(path, "rb") as f:
        head = f.read(6)
    return head[:1] == b"\x93" and head != b"\x93NUMPY"


def _read_legacy_int64_labels(path: str) -> np.ndarray:
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        magic = f.read(4)
        if magic != b"\x93\x00\x00\x00":
            raise ValueError(f"非预期旧格式 magic: {magic!r}")
        version = f.read(2)
        if version != b"\x01\x00":
            raise ValueError(f"非预期版本: {version!r}")
        header_len = struct.unpack("<H", f.read(2))[0]
        header = f.read(header_len)
        if len(header) != header_len:
            raise ValueError("头部截断")

    match = re.search(rb"shape': \((\d+),\)", header)
    if not match:
        raise ValueError("无法从头部解析 shape")
    expected_n = int(match.group(1))

    data_off = 4 + 2 + 2 + header_len
    data_bytes = size - data_off
    if data_bytes % 8 != 0:
        raise ValueError(f"数据区不是 8 字节对齐: {data_bytes}")
    actual_n = data_bytes // 8
    if actual_n != expected_n:
        raise ValueError(f"shape 不一致: header={expected_n}, file={actual_n}")

    with open(path, "rb") as f:
        f.seek(data_off)
        raw = f.read(data_bytes)
    return np.frombuffer(raw, dtype="<i8").copy()


def _write_standard_npy(path: str, arr: np.ndarray) -> None:
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        npy_format.write_array(f, arr, version=(1, 0), allow_pickle=False)
    os.replace(tmp, path)


def repair_file(path: str, backup: bool) -> None:
    if not _is_legacy_broken_npy(path):
        print(f"跳过（已是标准格式）: {path}")
        return

    labels = _read_legacy_int64_labels(path)
    if backup:
        bak = path + ".bak"
        if not os.path.exists(bak):
            import shutil

            shutil.copy2(path, bak)
    _write_standard_npy(path, labels)

    verify = np.load(path)
    if verify.shape != labels.shape or verify.dtype != np.int64:
        raise RuntimeError(f"修复后校验失败: {path}")
    print(
        f"已修复: {path}  shape={verify.shape}  "
        f"range=[{int(verify.min())}, {int(verify.max())}]"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="修复旧版标签 NPY 文件头")
    parser.add_argument(
        "paths",
        nargs="+",
        help="NPY 文件或目录（目录下匹配 *_labels.npy）",
    )
    parser.add_argument(
        "--backup",
        action="store_true",
        help="修复前将原文件重命名为 .bak",
    )
    args = parser.parse_args()

    targets: list[str] = []
    for p in args.paths:
        if os.path.isdir(p):
            for name in sorted(os.listdir(p)):
                if name.endswith("_labels.npy"):
                    targets.append(os.path.join(p, name))
        else:
            targets.append(p)

    if not targets:
        print("未找到待修复文件", file=sys.stderr)
        return 1

    for path in targets:
        repair_file(path, backup=args.backup)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
