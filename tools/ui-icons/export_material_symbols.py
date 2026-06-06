#!/usr/bin/env python3
"""从 Material Symbols Outlined SVG 批量导出 CloudSim UI 图标 PNG。"""

from __future__ import annotations

import re
import sys
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from io import BytesIO
from pathlib import Path

try:
    from PIL import Image
    import cairosvg
except ImportError:
    print("需要: pip install pillow cairosvg", file=sys.stderr)
    sys.exit(1)

# qrc basename -> Material Symbol 目录名
ICON_MAP: dict[str, str] = {
    "new_document": "note_add",
    "open_project": "folder_open",
    "save_project": "save",
    "open_model": "view_in_ar",
    "open_point_cloud": "scatter_plot",
    "undo": "undo",
    "redo": "redo",
    "delete": "delete",
    "clear": "delete_sweep",
    "add": "add",
    "rename": "drive_file_rename_outline",
    "duplicate": "content_copy",
    "run": "play_arrow",
    "stop": "stop",
    "export": "ios_share",
    "ptp": "place",
    "line": "timeline",
    "tcp_drag": "open_with",
    "apply": "check_circle",
    "reset": "restart_alt",
    "save_template": "bookmark_add",
    "load_template": "bookmark",
    "new_path_plan": "route",
    "pick_edge": "polyline",
    "pick_face": "texture",
    "discretize": "blur_linear",
    "refresh": "refresh",
    "fill_recipe": "auto_fix_high",
    "emit_program": "output",
    "view_mode": "3d_rotation",
    "object_select": "ads_click",
    "point_pick": "control_point",
    "line_pick": "show_chart",
    "face_pick": "select_all",
    "send": "send",
    "settings": "settings",
    "robot_placeholder": "precision_manufacturing",
    "connect": "link",
    "disconnect": "link_off",
    "read": "download",
    "write": "upload",
    "clear_log": "clear_all",
    "set_active": "star",
}

SVG_BASE = (
    "https://raw.githubusercontent.com/google/material-design-icons/master/"
    "symbols/web/{symbol}/materialsymbolsoutlined/{symbol}_24px.svg"
)

THEMES = {
    "light": "#424242",
    "dark": "#E0E0E0",
}

SIZES = (16, 24)


def fetch_svg(symbol: str) -> bytes:
    url = SVG_BASE.format(symbol=symbol)
    req = urllib.request.Request(url, headers={"User-Agent": "CloudSim-icon-export/1.0"})
    with urllib.request.urlopen(req, timeout=30) as resp:
        return resp.read()


def recolor_svg(svg_data: bytes, color: str) -> bytes:
    text = svg_data.decode("utf-8")
    text = re.sub(r'fill="#[0-9A-Fa-f]{3,8}"', f'fill="{color}"', text)
    text = re.sub(r'fill="currentColor"', f'fill="{color}"', text)
    if 'fill="' not in text and "<path" in text:
        text = text.replace("<path ", f'<path fill="{color}" ', 1)
    return text.encode("utf-8")


def svg_to_png(svg_data: bytes, size: int) -> Image.Image:
    png_bytes = cairosvg.svg2png(bytestring=svg_data, output_width=size, output_height=size)
    return Image.open(BytesIO(png_bytes)).convert("RGBA")


def main() -> int:
    repo_root = Path(__file__).resolve().parents[2]
    out_root = repo_root / "src" / "UI" / "CloudSimUiAssets" / "resources" / "icons"
    qrc_lines: list[str] = ['<RCC>', '    <qresource prefix="/cloudsim/icons">']

    failed: list[str] = []
    for basename, symbol in sorted(ICON_MAP.items()):
        try:
            svg_raw = fetch_svg(symbol)
        except (urllib.error.URLError, urllib.error.HTTPError) as exc:
            failed.append(f"{basename} ({symbol}): {exc}")
            continue

        for theme, color in THEMES.items():
            colored = recolor_svg(svg_raw, color)
            theme_dir = out_root / theme
            theme_dir.mkdir(parents=True, exist_ok=True)
            for size in SIZES:
                img = svg_to_png(colored, size)
                filename = f"{basename}_{size}.png"
                out_path = theme_dir / filename
                img.save(out_path, "PNG")
                qrc_lines.append(f'        <file alias="{theme}/{filename}">icons/{theme}/{filename}</file>')

    qrc_lines.append("    </qresource>")
    qrc_lines.append("</RCC>")

    qrc_path = repo_root / "src" / "UI" / "CloudSimUiAssets" / "resources" / "cloudsim_icons.qrc"
    qrc_path.parent.mkdir(parents=True, exist_ok=True)
    qrc_path.write_text("\n".join(qrc_lines) + "\n", encoding="utf-8")

    print(f"Exported {len(ICON_MAP) - len(failed)}/{len(ICON_MAP)} icons -> {out_root}")
    print(f"Wrote {qrc_path}")
    if failed:
        print("Failures:", file=sys.stderr)
        for line in failed:
            print(f"  {line}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
