# -*- coding: utf-8 -*-
"""Embed full_api_catalog.json as chunked C string literals (MSVC C2026-safe)."""
from pathlib import Path

json_path = Path(__file__).resolve().parent / "full_api_catalog.json"
out_path = Path(__file__).resolve().parents[3] / "src" / "UI" / "CloudSimPluginHost" / "source" / "Ai" / "AiApiCatalogEmbedded.cpp"
j = json_path.read_text(encoding="utf-8").strip()

# MSVC 单条字符串字面量上限约 16KB；分块拼接
chunk = 12000
parts = [j[i : i + chunk] for i in range(0, len(j), chunk)]
chunks_cpp = []
for idx, p in enumerate(parts):
    # 转义为普通字符串，避免 raw string 跨块问题
    esc = (
        p.replace("\\", "\\\\")
        .replace('"', '\\"')
        .replace("\r", "")
        .replace("\n", "\\n")
    )
    chunks_cpp.append(f'\t\t"{esc}"')

body = "\n".join(chunks_cpp)
cpp = (
    "/// @file AiApiCatalogEmbedded.cpp\n"
    "/// @brief 与 tools/ai-training/catalog/full_api_catalog.json 同步的嵌入 Catalog\n"
    "\n"
    '#include "Ai/AiApiCatalogEmbedded.h"\n'
    "\n"
    "QByteArray aiEmbeddedApiCatalogJson()\n"
    "{\n"
    "\tstatic const char kCatalog[] =\n"
    f"{body}\n"
    "\t;\n"
    "\treturn QByteArray(kCatalog);\n"
    "}\n"
)
out_path.write_text(cpp, encoding="utf-8", newline="\n")
text = out_path.read_text(encoding="utf-8")
assert "体素下采样" in text or "\\u" in text or "downsamplePointCloudVoxel" in text
print("ok chunks", len(parts), "bytes", len(j))
