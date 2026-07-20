#!/usr/bin/env python3
"""Detect and fix Chinese mojibake in CloudSim product sources.

Many legacy files contain irreversible U+FFFD corruption of original GBK text.
Strategy: scan for U+FFFD / known broken patterns; apply semantic rewrites from
clean Widget counterparts or context. Auto latin1->gbk only when it yields CJK.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _source_paths import ROOT, iter_sources  # noqa: E402


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


def has_mojibake(text: str) -> bool:
    if "\ufffd" in text:
        return True
    # High-byte latin fragments without CJK on a comment/string line
    for line in text.splitlines():
        if not any(ord(c) > 127 for c in line):
            continue
        cjk = sum(1 for c in line if "\u4e00" <= c <= "\u9fff")
        high_latin = sum(1 for c in line if 0x80 <= ord(c) <= 0xFF)
        if "\ufffd" in line:
            return True
        if high_latin >= 4 and cjk == 0 and ("//" in line or "/*" in line or '"' in line or "///" in line):
            return True
    return False


def try_recover_line(line: str) -> str | None:
    core = line.rstrip("\r\n")
    nl = line[len(core) :]
    for enc_in, enc_out in (("latin1", "gbk"), ("cp1252", "gbk")):
        try:
            recovered = core.encode(enc_in).decode(enc_out)
        except (UnicodeEncodeError, UnicodeDecodeError):
            continue
        if "\ufffd" in recovered:
            continue
        if any("\u4e00" <= c <= "\u9fff" for c in recovered):
            return recovered + nl
    return None


# Path relative to CloudSim root -> list of (old, new) exact replacements
SEMANTIC_FIXES: dict[str, list[tuple[str, str]]] = {
    "src/UI/Widget/inc/widget_global.h": [
        (
            'qCritical() << "',
            None,  # handled specially
        ),
    ],
}


def fix_widget_global(text: str) -> str:
    # Replace any garbled catch message with correct Chinese
    text = re.sub(
        r'qCritical\(\)\s*<<\s*"[^"]*"\s*<<\s*__FILE__',
        'qCritical() << "空指针、野指针：" << __FILE__',
        text,
        count=1,
    )
    return text


def fix_host_qwidget_viewer(text: str) -> str:
    text = re.sub(
        r"/\*{5,}[\s\S]*?\*{5,}/",
        "/// @file QWidgetViewer.h\n"
        "/// @brief OpenGL/OSG 嵌入 Qt；输入事件转发 osgViewer，供 GraphicsWindowQt1 使用",
        text,
        count=1,
    )
    replacements = [
        ("#include <QtCore>   // 需要QMutexLocker声明", None),
        (r"#include <QtCore>\s*//[^\n]*", "#include <QtCore>"),
        (r"#include <QtWidgets>\s*//[^\n]*", "#include <QtWidgets>"),
        (r"#include <QGestureEvent>\s*//[^\n]*", "#include <QGestureEvent>"),
        (r"#include <QCursor>\s*//[^\n]*", "#include <QCursor>"),
        (
            r"///[^\n]*OpenGL[^\n]*\n",
            "/// OpenGL/OSG 嵌入 Qt；输入事件转发 osgViewer，供 GraphicsWindowQt1 使用\n",
        ),
    ]
    # Simpler: strip trailing garbled comments on those includes
    text = re.sub(r"(#include <QtCore>)\s*//[^\n]*", r"\1", text)
    text = re.sub(r"(#include <QtWidgets>)\s*//[^\n]*", r"\1", text)
    text = re.sub(r"(#include <QGestureEvent>)\s*//[^\n]*", r"\1", text)
    text = re.sub(r"(#include <QCursor>)\s*//[^\n]*", r"\1", text)
    # Replace class doc line if it still has mojibake
    lines = text.splitlines(keepends=True)
    out = []
    for line in lines:
        if line.lstrip().startswith("///") and ("OpenGL" in line or "OSG" in line or "\ufffd" in line):
            if has_mojibake(line) or "\ufffd" in line:
                out.append(
                    "/// OpenGL/OSG 嵌入 Qt；输入事件转发 osgViewer，供 GraphicsWindowQt1 使用\n"
                )
                continue
        out.append(line)
    return "".join(out)


def fix_host_graphics_window_h(text: str) -> str:
    text = re.sub(r"(#include <QtCore>)\s*//[^\n]*", r"\1", text)
    text = re.sub(r"(#include <QtWidgets>)\s*//[^\n]*", r"\1", text)
    text = re.sub(r"(#include <QGestureEvent>)\s*//[^\n]*", r"\1", text)
    text = re.sub(r"(#include <QCursor>)\s*//[^\n]*", r"\1", text)
    lines = text.splitlines(keepends=True)
    out = []
    for line in lines:
        s = line.lstrip()
        if s.startswith("///") and ("OSG" in line or "\ufffd" in line or has_mojibake(line)):
            if has_mojibake(line) or "\ufffd" in line:
                out.append(
                    "/// OSG 图形窗口适配：实现 osgViewer::GraphicsWindow，与 QWidgetViewer 同步尺寸与事件\n"
                )
                continue
        if s.startswith("//") and has_mojibake(line):
            # 延迟尺寸更新路径
            out.append("\t/// 同步 traits 宽高并触发 resized\n")
            continue
        out.append(line)
    return "".join(out)


def fix_host_graphics_window_cpp(text: str) -> str:
    # Line-level semantic map for known destructor / release comments
    mapping = [
        (re.compile(r"//[^\n]*QWidgetViewer[^\n]*"), "// 先断开与 QWidgetViewer 的关联"),
        (re.compile(r"//[^\n]*Ұ[^\n]*|//[^\n]*野指针|//[^\n]*\ufffd[^\n]*ָ"), "// 防止野指针"),
        (re.compile(r"//[^\n]*Ƿ[^\n]*|//[^\n]*\ufffd[^\n]*ر"), "// 安全关闭（无论是否已关闭）"),
        (re.compile(r"//[^\n]*ʧ[^\n]*|//[^\n]*失败状态"), "// 保持失败状态"),
    ]
    lines = text.splitlines(keepends=True)
    out = []
    for line in lines:
        if "//" not in line or not (has_mojibake(line) or "\ufffd" in line):
            out.append(line)
            continue
        replaced = False
        for rx, repl in mapping:
            if rx.search(line):
                # preserve indent before //
                m = re.match(r"^(\s*)//.*$", line.rstrip("\r\n"))
                if m:
                    out.append(m.group(1) + repl + ("\n" if line.endswith("\n") else ""))
                    replaced = True
                    break
        if not replaced:
            # Drop irreparable trailing comment, keep code
            code = re.sub(r"\s*//.*$", "", line.rstrip("\r\n"))
            out.append(code + ("\n" if line.endswith("\n") else ""))
        # second mapping for 防止野指针 on same-line after code
    text2 = "".join(out)
    # Fix inline: `_widget = nullptr;  // garbled`
    text2 = re.sub(
        r"(_widget\s*=\s*nullptr;)\s*//[^\n]*",
        r"\1  // 防止野指针",
        text2,
    )
    text2 = re.sub(
        r"(return false;)\s*//[^\n]*",
        r"\1 // 保持失败状态",
        text2,
    )
    return text2


def fix_file(path: Path, text: str) -> str:
    rel = path.relative_to(ROOT).as_posix()
    if rel.endswith("widget_global.h"):
        return fix_widget_global(text)
    if rel.endswith("CloudSimHost/inc/osg/QWidgetViewer.h"):
        return fix_host_qwidget_viewer(text)
    if rel.endswith("CloudSimHost/inc/osg/GraphicsWindowQt1.h"):
        return fix_host_graphics_window_h(text)
    if rel.endswith("CloudSimHost/source/osg/GraphicsWindowQt1.cpp"):
        return fix_host_graphics_window_cpp(text)

    # Generic: try line recovery; drop irreparable comment tails
    lines = text.splitlines(keepends=True)
    out = []
    for line in lines:
        if not has_mojibake(line) and "\ufffd" not in line:
            out.append(line)
            continue
        recovered = try_recover_line(line)
        if recovered is not None:
            out.append(recovered)
            continue
        if "//" in line:
            code = re.sub(r"\s*//.*$", "", line.rstrip("\r\n"))
            out.append(code + ("\n" if line.endswith("\n") else ""))
        elif line.lstrip().startswith("/*") or line.lstrip().startswith("*"):
            continue  # drop garbled block lines
        else:
            out.append(line)
    return "".join(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--report", type=Path, default=None)
    args = ap.parse_args()

    fixed: list[str] = []
    flagged: list[str] = []

    for path in iter_sources():
        orig = read_text(path)
        if not has_mojibake(orig) and "\ufffd" not in orig:
            continue
        new = fix_file(path, orig)
        if new != orig:
            fixed.append(str(path.relative_to(ROOT)))
            print(f"{'DRY ' if args.dry_run else ''}FIX: {path.relative_to(ROOT)}")
            if not args.dry_run:
                write_text(path, new)
            check = new
        else:
            check = orig
        if has_mojibake(check) or "\ufffd" in check:
            flagged.append(str(path.relative_to(ROOT)))

    print(f"fixed={len(fixed)} still_flagged={len(flagged)}")
    for f in flagged:
        print(f"FLAG: {f}")
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(
            "\n".join(["# fixed", *fixed, "", "# still_flagged", *flagged]) + "\n",
            encoding="utf-8",
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
