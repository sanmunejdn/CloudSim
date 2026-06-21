#!/usr/bin/env python3
"""离线生成 CloudSim 应用 Logo（云轮廓 + 三维坐标轴）。"""

from __future__ import annotations

import math
from pathlib import Path

from PIL import Image, ImageDraw

LOGO_SIZES = (16, 24, 32, 48, 64, 128, 256)
THEMES = {"light": "#424242", "dark": "#E0E0E0"}


def hex_rgb(color: str) -> tuple[int, int, int]:
    color = color.lstrip("#")
    return tuple(int(color[i : i + 2], 16) for i in (0, 2, 4))


def scale(v: float, size: int, base: int = 64) -> float:
    return v * size / base


def stroke_w(size: int, base_stroke: float = 2.0) -> int:
    return max(1, int(round(base_stroke * size / 64)))


def draw_cloud(draw: ImageDraw.ImageDraw, size: int, rgb: tuple[int, int, int], sw: int) -> None:
    """绘制三段弧拼接的云轮廓。"""
    s = lambda v: scale(v, size)
    cx = size / 2
    # 云中心略偏上
    cy = size / 2 - s(2)

    # 左下弧
    draw.arc(
        [cx - s(20), cy - s(6), cx - s(2), cy + s(12)],
        start=120, end=250, fill=rgb, width=sw,
    )
    # 顶弧（主体）
    draw.arc(
        [cx - s(18), cy - s(16), cx + s(10), cy + s(2)],
        start=200, end=350, fill=rgb, width=sw,
    )
    # 右弧
    draw.arc(
        [cx - s(6), cy - s(10), cx + s(18), cy + s(8)],
        start=290, end=60, fill=rgb, width=sw,
    )
    # 底部封口线
    draw.line(
        [(cx - s(19), cy + s(8)), (cx + s(16), cy + s(5))],
        fill=rgb, width=sw,
    )


def draw_axes(draw: ImageDraw.ImageDraw, size: int, rgb: tuple[int, int, int], sw: int) -> None:
    """绘制三维坐标轴（X 右下、Y 左下、Z 上）。"""
    s = lambda v: scale(v, size)
    cx = size / 2
    cy = size / 2 + s(4)
    ax_len = s(10)

    # X 轴（向右下）
    x_ex = cx + ax_len * 0.85
    x_ey = cy + ax_len * 0.5
    draw.line([(cx, cy), (x_ex, x_ey)], fill=rgb, width=sw)
    a = s(2.5)
    draw.polygon([
        (x_ex, x_ey),
        (x_ex - a * 0.9, x_ey - a * 0.35),
        (x_ex - a * 0.35, x_ey - a * 0.9),
    ], fill=rgb)

    # Y 轴（向左下）
    y_ex = cx - ax_len * 0.85
    y_ey = cy + ax_len * 0.5
    draw.line([(cx, cy), (y_ex, y_ey)], fill=rgb, width=sw)
    draw.polygon([
        (y_ex, y_ey),
        (y_ex + a * 0.9, y_ey - a * 0.35),
        (y_ex + a * 0.35, y_ey - a * 0.9),
    ], fill=rgb)

    # Z 轴（向上）
    z_ex = cx
    z_ey = cy - ax_len
    draw.line([(cx, cy), (z_ex, z_ey)], fill=rgb, width=sw)
    draw.polygon([
        (z_ex, z_ey),
        (z_ex - a * 0.45, z_ey + a * 0.9),
        (z_ex + a * 0.45, z_ey + a * 0.9),
    ], fill=rgb)

    # 原点圆点
    dot_r = max(1, s(1.5))
    draw.ellipse([cx - dot_r, cy - dot_r, cx + dot_r, cy + dot_r], fill=rgb)


def draw_logo(size: int, color: str) -> Image.Image:
    img = Image.new("RGBA", (size, size), (0, 0, 0, 0))
    draw = ImageDraw.Draw(img)
    rgb = hex_rgb(color)
    sw = stroke_w(size)

    draw_cloud(draw, size, rgb, sw)
    draw_axes(draw, size, rgb, sw)

    return img


def main() -> int:
    repo_root = Path(__file__).resolve().parents[2]
    out_root = repo_root / "src" / "UI" / "CloudSimUiAssets" / "resources" / "logo"

    for theme, color in THEMES.items():
        for size in LOGO_SIZES:
            img = draw_logo(size, color)
            out_path = out_root / f"{theme}" / f"cloudsim_logo_{size}.png"
            out_path.parent.mkdir(parents=True, exist_ok=True)
            img.save(out_path, "PNG")

    print(f"Logo PNGs generated: {out_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
