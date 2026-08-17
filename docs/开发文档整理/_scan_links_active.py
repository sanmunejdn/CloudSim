# -*- coding: utf-8 -*-
from __future__ import annotations

import re
from collections import defaultdict
from pathlib import Path

CLOUD = Path(r"d:/Project/VSprogram/CGAL5.5.2/CloudSim")
SKIP_PARTS = {
    "third_party",
    "onecad_src",
    "node_modules",
    ".tools",
    "ported",
    "_archive",  # archive links fixed opportunistically; not blocking
}

md_files: list[Path] = []
# docs active
for p in (CLOUD / "docs").rglob("*.md"):
    if any(x in p.parts for x in SKIP_PARTS):
        continue
    md_files.append(p)
# src guides
for p in (CLOUD / "src").rglob("*.md"):
    if any(x in p.parts for x in ("third_party", "onecad_src", "ported", "node_modules")):
        continue
    if p.name in {"DEVELOPER_GUIDE.md", "README.md", "CONVENTIONS.md", "ORIGIN.md"}:
        md_files.append(p)
# web guide
web_guide = CLOUD / "web" / "cloudsim-web-ui" / "DEVELOPER_GUIDE.md"
if web_guide.exists():
    md_files.append(web_guide)

link_re = re.compile(r"\[([^\]]*)\]\(([^)]+)\)")
broken = defaultdict(list)
checked = 0
for md in md_files:
    text = md.read_text(encoding="utf-8-sig", errors="replace")
    for m in link_re.finditer(text):
        url = m.group(2).strip()
        if url.startswith(("http://", "https://", "mailto:", "#", "file:")):
            continue
        path_part = url.split()[0].strip("\"'")
        path_part = path_part.split("#")[0]
        if not path_part:
            continue
        target = (md.parent / path_part).resolve()
        checked += 1
        if not target.exists():
            rel = str(md.relative_to(CLOUD)).replace("\\", "/")
            broken[rel].append(path_part)

out = CLOUD / "docs" / "开发文档整理" / "_link_scan_active.txt"
lines = [f"files={len(md_files)} checked={checked} broken_files={len(broken)} broken_links={sum(len(v) for v in broken.values())}", ""]
for src in sorted(broken):
    lines.append(f"--- {src}")
    for link in broken[src]:
        lines.append(f"  {link}")
out.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(lines[0])
print(f"wrote {out}")
