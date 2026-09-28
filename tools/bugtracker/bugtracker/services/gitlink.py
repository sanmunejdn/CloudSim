"""解析 commit message 中的 bug 引用：fix #ID 联动修复，ref #ID / 裸 #ID 仅关联。"""
from __future__ import annotations

import re

FIX_RE = re.compile(
    r"(?:fix|fixed|fixes|close|closes|closed|resolve|resolves|resolved)"
    r"\s+#(\d+)", re.IGNORECASE)
REF_RE = re.compile(r"(?:ref|refs|see|bug)\s+#(\d+)", re.IGNORECASE)
ANY_RE = re.compile(r"#(\d+)")


def parse_commit_message(message: str) -> tuple[set[int], set[int]]:
    """返回 (fix 集合, ref 集合)；fix 优先，ref 不含 fix。"""
    text = message or ""
    fix_ids = {int(i) for i in FIX_RE.findall(text)}
    ref_ids = {int(i) for i in REF_RE.findall(text)}
    ref_ids |= {int(i) for i in ANY_RE.findall(text)}
    ref_ids -= fix_ids
    return fix_ids, ref_ids
