#!/usr/bin/env python3
"""离线生成 CloudSim UI 单色 PNG 图标（Material Outlined 风格几何近似）。"""

from __future__ import annotations

import math
from pathlib import Path

from PIL import Image, ImageDraw

ICON_BASENAMES: list[str] = [
    "new_document", "open_project", "save_project", "open_model", "open_point_cloud",
    "undo", "redo", "delete", "clear", "add", "rename", "duplicate",
    "run", "stop", "export", "ptp", "line", "tcp_drag",
    "apply", "reset", "save_template", "load_template", "new_path_plan",
    "pick_edge", "pick_face", "discretize", "refresh", "fill_recipe", "emit_program",
    "view_mode", "object_select", "point_pick", "line_pick", "face_pick",
    "send", "settings", "robot_placeholder", "connect", "disconnect",
    "read", "write", "clear_log", "set_active",
    "focus_camera", "wireframe", "screenshot",
    "close", "dock_float",
]

THEMES = {"light": "#424242", "dark": "#E0E0E0"}
SIZES = (16, 24)
STROKE = 2.0


def hex_rgb(color: str) -> tuple[int, int, int]:
    color = color.lstrip("#")
    return tuple(int(color[i : i + 2], 16) for i in (0, 2, 4))


def canvas(size: int, color: str) -> tuple[Image.Image, ImageDraw.ImageDraw, tuple[int, int, int]]:
    img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    return img, draw, hex_rgb(color)


def scale(v: float, size: int, base: int = 24) -> float:
    return v * size / base


def stroke_w(size: int) -> int:
    return max(1, int(round(STROKE * size / 24)))


def draw_icon(name: str, size: int, color: str) -> Image.Image:
    img, draw, rgb = canvas(size, color)
    s = lambda v: scale(v, size)
    sw = stroke_w(size)
    pad = s(3)
    box = (pad, pad, size - pad, size - pad)

    if name == "run":
        x0, y0, x1, y1 = box
        tri = [(x0 + s(2), (y0 + y1) / 2), (x1 - s(1), y0 + s(2)), (x1 - s(1), y1 - s(2))]
        draw.polygon(tri, fill=rgb)
    elif name == "stop":
        draw.rounded_rectangle(box, radius=s(2), fill=rgb)
    elif name == "add":
        cx = cy = size / 2
        draw.line([(cx, pad), (cx, size - pad)], fill=rgb, width=sw)
        draw.line([(pad, cy), (size - pad, cy)], fill=rgb, width=sw)
    elif name == "delete":
        draw.line([(pad, pad), (size - pad, size - pad)], fill=rgb, width=sw)
        draw.line([(size - pad, pad), (pad, size - pad)], fill=rgb, width=sw)
    elif name == "undo":
        draw.arc([pad, pad, size - pad, size - pad], start=120, end=300, fill=rgb, width=sw)
        draw.polygon([(pad + s(1), s(8)), (pad + s(6), s(4)), (pad + s(6), s(12))], fill=rgb)
    elif name == "redo":
        draw.arc([pad, pad, size - pad, size - pad], start=-60, end=120, fill=rgb, width=sw)
        draw.polygon([(size - pad - s(1), s(8)), (size - pad - s(6), s(4)), (size - pad - s(6), s(12))], fill=rgb)
    elif name == "save_project":
        draw.rounded_rectangle([s(4), s(3), s(19), s(20)], radius=s(1), outline=rgb, width=sw)
        draw.rectangle([s(7), s(3), s(16), s(7)], fill=rgb)
        draw.rectangle([s(7), s(10), s(16), s(17)], outline=rgb, width=sw)
    elif name == "open_project":
        draw.polygon([(s(3), s(9)), (s(10), s(9)), (s(12), s(6)), (s(21), s(6)), (s(21), s(19)), (s(3), s(19))], outline=rgb, width=sw)
        draw.line([(s(3), s(11)), (s(21), s(11))], fill=rgb, width=sw)
    elif name == "new_document":
        draw.rounded_rectangle([s(5), s(3), s(19), s(21)], radius=s(1), outline=rgb, width=sw)
        draw.line([(size / 2, s(8)), (size / 2, s(16))], fill=rgb, width=sw)
        draw.line([(s(9), s(12)), (s(15), s(12))], fill=rgb, width=sw)
    elif name == "open_model":
        draw.polygon([(s(12), s(4)), (s(20), s(10)), (s(17), s(20)), (s(7), s(20)), (s(4), s(10))], outline=rgb, width=sw)
    elif name == "open_point_cloud":
        for cx, cy in [(s(8), s(8)), (s(15), s(7)), (s(11), s(14)), (s(17), s(16)), (s(6), s(16))]:
            r = s(1.8)
            draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=rgb)
    elif name == "clear":
        draw.arc([s(5), s(5), s(19), s(19)], start=30, end=330, fill=rgb, width=sw)
        draw.line([(s(8), s(16)), (s(16), s(8))], fill=rgb, width=sw)
    elif name == "rename":
        draw.line([(s(4), s(18)), (s(14), s(8))], fill=rgb, width=sw)
        draw.polygon([(s(13), s(7)), (s(18), s(7)), (s(18), s(12))], outline=rgb, width=sw)
    elif name == "duplicate":
        draw.rounded_rectangle([s(3), s(6), s(14), s(17)], radius=s(1), outline=rgb, width=sw)
        draw.rounded_rectangle([s(10), s(3), s(21), s(14)], radius=s(1), outline=rgb, width=sw)
    elif name == "export":
        draw.line([(size / 2, s(4)), (size / 2, s(14))], fill=rgb, width=sw)
        draw.polygon([(s(9), s(10)), (size / 2, s(16)), (s(15), s(10))], fill=rgb)
        draw.line([(s(5), s(19)), (s(19), s(19))], fill=rgb, width=sw)
    elif name == "ptp":
        r = s(3)
        draw.ellipse([size / 2 - r, size / 2 - r, size / 2 + r, size / 2 + r], fill=rgb)
        draw.ellipse([size / 2 - s(1), size / 2 - s(1), size / 2 + s(1), size / 2 + s(1)], fill=(0, 0, 0, 0))
    elif name == "line":
        draw.line([(s(4), s(18)), (s(20), s(6))], fill=rgb, width=sw)
        for t in (0.0, 0.5, 1.0):
            cx = s(4) + (s(20) - s(4)) * t
            cy = s(18) + (s(6) - s(18)) * t
            r = s(1.5)
            draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=rgb)
    elif name == "tcp_drag":
        draw.line([(s(6), s(6)), (s(18), s(18))], fill=rgb, width=sw)
        draw.line([(s(18), s(6)), (s(6), s(18))], fill=rgb, width=sw)
        draw.ellipse([s(4), s(4), s(8), s(8)], outline=rgb, width=sw)
        draw.ellipse([s(16), s(16), s(20), s(20)], outline=rgb, width=sw)
    elif name == "apply":
        r = s(9)
        cx = cy = size / 2
        draw.ellipse([cx - r, cy - r, cx + r, cy + r], outline=rgb, width=sw)
        draw.line([(s(8), s(12)), (s(11), s(15)), (s(17), s(9))], fill=rgb, width=sw)
    elif name == "reset":
        draw.arc([pad, pad, size - pad, size - pad], start=45, end=315, fill=rgb, width=sw)
        draw.polygon([(size / 2, s(4)), (size / 2 + s(4), s(8)), (size / 2 - s(4), s(8))], fill=rgb)
    elif name == "save_template":
        draw.rounded_rectangle([s(6), s(4), s(18), s(20)], radius=s(1), outline=rgb, width=sw)
        draw.line([(s(6), s(8)), (s(18), s(8))], fill=rgb, width=sw)
        draw.polygon([(size / 2, s(11)), (s(15), s(14)), (s(9), s(14))], fill=rgb)
    elif name == "load_template":
        draw.rounded_rectangle([s(6), s(4), s(18), s(20)], radius=s(1), outline=rgb, width=sw)
        draw.line([(s(6), s(8)), (s(18), s(8))], fill=rgb, width=sw)
        draw.polygon([(size / 2, s(15)), (s(15), s(11)), (s(9), s(11))], fill=rgb)
    elif name == "new_path_plan":
        draw.line([(s(4), s(16)), (s(10), s(8)), (s(16), s(14)), (s(20), s(6))], fill=rgb, width=sw)
    elif name == "pick_edge":
        draw.line([(s(4), s(18)), (s(10), s(6)), (s(16), s(16)), (s(20), s(8))], fill=rgb, width=sw)
    elif name == "pick_face":
        draw.polygon([(s(12), s(4)), (s(20), s(9)), (s(17), s(19)), (s(7), s(19)), (s(4), s(9))], outline=rgb, width=sw)
    elif name == "discretize":
        for i in range(4):
            y = s(6 + i * 4)
            draw.line([(s(5), y), (s(19), y)], fill=rgb, width=max(1, sw - 1))
    elif name == "refresh":
        draw.arc([pad, pad, size - pad, size - pad], start=30, end=330, fill=rgb, width=sw)
        draw.polygon([(s(16), s(5)), (s(19), s(9)), (s(14), s(9))], fill=rgb)
    elif name == "fill_recipe":
        draw.line([(s(5), s(12)), (s(10), s(7)), (s(14), s(13)), (s(19), s(8))], fill=rgb, width=sw)
        draw.ellipse([s(4), s(14), s(8), s(18)], outline=rgb, width=sw)
    elif name == "emit_program":
        draw.rounded_rectangle([s(4), s(6), s(16), s(18)], radius=s(1), outline=rgb, width=sw)
        draw.polygon([(s(15), s(10)), (s(20), s(12)), (s(15), s(14))], fill=rgb)
    elif name == "view_mode":
        draw.polygon([(s(12), s(4)), (s(20), s(18)), (s(4), s(18))], outline=rgb, width=sw)
        draw.line([(s(8), s(14)), (s(16), s(14))], fill=rgb, width=sw)
    elif name == "object_select":
        draw.line([(s(5), s(18)), (s(12), s(6))], fill=rgb, width=sw)
        draw.ellipse([s(13), s(5), s(19), s(11)], outline=rgb, width=sw)
    elif name == "point_pick":
        r = s(2)
        draw.ellipse([size / 2 - r, size / 2 - r, size / 2 + r, size / 2 + r], fill=rgb)
        draw.line([(size / 2, pad), (size / 2, size - pad)], fill=rgb, width=1)
        draw.line([(pad, size / 2), (size - pad, size / 2)], fill=rgb, width=1)
    elif name == "line_pick":
        draw.line([(s(4), s(18)), (s(20), s(6))], fill=rgb, width=sw)
        draw.rectangle([s(3), s(3), s(8), s(8)], outline=rgb, width=sw)
    elif name == "face_pick":
        draw.rectangle([s(5), s(5), s(19), s(19)], outline=rgb, width=sw)
        draw.line([(s(5), s(10)), (s(19), s(10))], fill=rgb, width=1)
        draw.line([(s(10), s(5)), (s(10), s(19))], fill=rgb, width=1)
    elif name == "send":
        draw.polygon([(s(4), s(6)), (s(4), s(18)), (s(18), s(12))], fill=rgb)
    elif name == "settings":
        cx = cy = size / 2
        r1, r2 = s(7), s(3)
        draw.ellipse([cx - r1, cy - r1, cx + r1, cy + r1], outline=rgb, width=sw)
        draw.ellipse([cx - r2, cy - r2, cx + r2, cy + r2], fill=rgb)
        for deg in range(0, 360, 45):
            rad = math.radians(deg)
            x0 = cx + math.cos(rad) * s(5)
            y0 = cy + math.sin(rad) * s(5)
            x1 = cx + math.cos(rad) * s(8)
            y1 = cy + math.sin(rad) * s(8)
            draw.line([(x0, y0), (x1, y1)], fill=rgb, width=sw)
    elif name == "robot_placeholder":
        draw.rounded_rectangle([s(7), s(5), s(17), s(14)], radius=s(2), outline=rgb, width=sw)
        draw.line([(s(12), s(14)), (s(12), s(18))], fill=rgb, width=sw)
        draw.line([(s(8), s(18)), (s(16), s(18))], fill=rgb, width=sw)
        draw.ellipse([s(9), s(7), s(11), s(9)], fill=rgb)
        draw.ellipse([s(13), s(7), s(15), s(9)], fill=rgb)
    elif name == "connect":
        draw.arc([s(3), s(8), s(14), s(20)], start=300, end=120, fill=rgb, width=sw)
        draw.arc([s(10), s(4), s(21), s(16)], start=120, end=300, fill=rgb, width=sw)
    elif name == "disconnect":
        draw.arc([s(3), s(8), s(14), s(20)], start=300, end=120, fill=rgb, width=sw)
        draw.arc([s(10), s(4), s(21), s(16)], start=120, end=300, fill=rgb, width=sw)
        draw.line([(s(9), s(9)), (s(15), s(15))], fill=rgb, width=sw)
    elif name == "read":
        draw.line([(size / 2, s(4)), (size / 2, s(12))], fill=rgb, width=sw)
        draw.polygon([(s(9), s(10)), (size / 2, s(16)), (s(15), s(10))], fill=rgb)
        draw.line([(s(5), s(19)), (s(19), s(19))], fill=rgb, width=sw)
    elif name == "write":
        draw.line([(size / 2, s(12)), (size / 2, s(4))], fill=rgb, width=sw)
        draw.polygon([(s(9), s(6)), (size / 2, s(10)), (s(15), s(6))], fill=rgb)
        draw.line([(s(5), s(19)), (s(19), s(19))], fill=rgb, width=sw)
    elif name == "clear_log":
        draw.rounded_rectangle([s(4), s(6), s(20), s(18)], radius=s(1), outline=rgb, width=sw)
        draw.line([(s(7), s(9)), (s(17), s(15))], fill=rgb, width=sw)
        draw.line([(s(17), s(9)), (s(7), s(15))], fill=rgb, width=sw)
    elif name == "set_active":
        cx, cy = size / 2, size / 2
        for i in range(5):
            ang = math.radians(-90 + i * 144)
            x = cx + math.cos(ang) * s(6)
            y = cy + math.sin(ang) * s(6)
            r = s(2)
            draw.polygon(
                [(cx + math.cos(ang) * s(8), cy + math.sin(ang) * s(8)),
                 (x + s(1.5), y), (x - s(1.5), y)],
                fill=rgb,
            )
    elif name == "focus_camera":
        # 十字准心 + 四角括号
        cx, cy = size / 2, size / 2
        r = s(7)
        # 中心圆点
        dot = s(1.5)
        draw.ellipse([cx - dot, cy - dot, cx + dot, cy + dot], fill=rgb)
        # 十字线
        draw.line([(cx, cy - s(4)), (cx, cy + s(4))], fill=rgb, width=1)
        draw.line([(cx - s(4), cy), (cx + s(4), cy)], fill=rgb, width=1)
        # 四角括号
        corner = s(2.5)
        gap = s(3)
        # 左上
        draw.line([(cx - r, cy - r + corner), (cx - r, cy - r)], fill=rgb, width=sw)
        draw.line([(cx - r, cy - r), (cx - r + corner, cy - r)], fill=rgb, width=sw)
        # 右上
        draw.line([(cx + r - corner, cy - r), (cx + r, cy - r)], fill=rgb, width=sw)
        draw.line([(cx + r, cy - r), (cx + r, cy - r + corner)], fill=rgb, width=sw)
        # 左下
        draw.line([(cx - r, cy + r - corner), (cx - r, cy + r)], fill=rgb, width=sw)
        draw.line([(cx - r, cy + r), (cx - r + corner, cy + r)], fill=rgb, width=sw)
        # 右下
        draw.line([(cx + r - corner, cy + r), (cx + r, cy + r)], fill=rgb, width=sw)
        draw.line([(cx + r, cy + r - corner), (cx + r, cy + r)], fill=rgb, width=sw)
    elif name == "wireframe":
        # 立方体线框（等轴测透视）
        s0 = s(5)
        s1 = s(19)
        # 前面矩形
        draw.rectangle([s(5), s(8), s(15), s(18)], outline=rgb, width=sw)
        # 后面矩形（偏移）
        draw.rectangle([s(9), s(4), s(19), s(14)], outline=rgb, width=sw)
        # 连接线
        draw.line([(s(5), s(8)), (s(9), s(4))], fill=rgb, width=sw)
        draw.line([(s(15), s(8)), (s(19), s(4))], fill=rgb, width=sw)
        draw.line([(s(5), s(18)), (s(9), s(14))], fill=rgb, width=sw)
        draw.line([(s(15), s(18)), (s(19), s(14))], fill=rgb, width=sw)
    elif name == "screenshot":
        # 相机图标
        # 机身
        draw.rounded_rectangle([s(4), s(8), s(20), s(18)], radius=s(1.5), outline=rgb, width=sw)
        # 镜头凸起
        draw.polygon([(s(8), s(8)), (s(10), s(5)), (s(14), s(5)), (s(16), s(8))], outline=rgb, width=sw)
        # 镜头圆
        cx_lens = size / 2
        cy_lens = s(13)
        r_lens = s(2.5)
        draw.ellipse([cx_lens - r_lens, cy_lens - r_lens, cx_lens + r_lens, cy_lens + r_lens], outline=rgb, width=sw)
        # 闪光灯小圆
        draw.ellipse([s(16), s(9.5), s(17.5), s(11)], fill=rgb)
    elif name == "close":
        draw.line([(pad, pad), (size - pad, size - pad)], fill=rgb, width=sw)
        draw.line([(size - pad, pad), (pad, size - pad)], fill=rgb, width=sw)
    elif name == "dock_float":
        # 窗体 + 右上角弹出箭头（open_in_new）
        draw.rounded_rectangle([s(4), s(7), s(15), s(18)], radius=s(1.5), outline=rgb, width=sw)
        draw.line([(s(12), s(5)), (s(19), s(5))], fill=rgb, width=sw)
        draw.line([(s(19), s(5)), (s(19), s(12))], fill=rgb, width=sw)
        draw.line([(s(13), s(11)), (s(19), s(5))], fill=rgb, width=sw)
    else:
        draw.ellipse(box, outline=rgb, width=sw)

    return img


def write_qrc(qrc_path: Path, entries: list[tuple[str, str]]) -> None:
    lines = ["<RCC>", '    <qresource prefix="/cloudsim/icons">']
    for alias, rel in entries:
        lines.append(f'        <file alias="{alias}">icons/{rel}</file>')
    lines.extend(["    </qresource>", "</RCC>", ""])
    qrc_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    repo_root = Path(__file__).resolve().parents[2]
    out_root = repo_root / "src" / "UI" / "CloudSimUiAssets" / "resources" / "icons"
    qrc_entries: list[tuple[str, str]] = []

    for basename in ICON_BASENAMES:
        for theme, color in THEMES.items():
            for size in SIZES:
                img = draw_icon(basename, size, color)
                rel = f"{theme}/{basename}_{size}.png"
                out_path = out_root / rel
                out_path.parent.mkdir(parents=True, exist_ok=True)
                img.save(out_path, "PNG")
                qrc_entries.append((rel, rel))

    qrc_path = repo_root / "src" / "UI" / "CloudSimUiAssets" / "resources" / "cloudsim_icons.qrc"
    write_qrc(qrc_path, qrc_entries)
    print(f"Generated {len(ICON_BASENAMES)} icons x {len(THEMES)} themes x {len(SIZES)} sizes")
    print(f"Output: {out_root}")
    print(f"QRC: {qrc_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
