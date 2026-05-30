#!/usr/bin/env python3
"""生成 geometry.recognize 多模态训练样本（合成视口风格 PNG + dataset.jsonl）。"""
from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOMAIN_DIR = ROOT / "domains" / "geometry.recognize"
IMAGES_DIR = DOMAIN_DIR / "images"
DATASET_PATH = DOMAIN_DIR / "dataset.jsonl"

INSTRUCTION = "识别图中的基本体类型与大致尺寸"
LABELS = {
    "box": "长方体",
    "cylinder": "圆柱体",
    "cone": "圆锥体",
    "sphere": "球体",
}


def _rand_dim(lo: float, hi: float) -> float:
    return round(random.uniform(lo, hi), 1)


def sample_spec(primitive: str) -> dict:
    if primitive == "box":
        dims = {
            "length": _rand_dim(40, 200),
            "width": _rand_dim(30, 120),
            "height": _rand_dim(20, 80),
        }
    elif primitive in ("cylinder", "cone"):
        dims = {"radius": _rand_dim(15, 80), "height": _rand_dim(40, 160)}
    elif primitive == "sphere":
        dims = {"radius": _rand_dim(20, 90)}
    else:
        raise ValueError(primitive)
    return {
        "primitive": primitive,
        "label": LABELS[primitive],
        "dimensions_mm": dims,
        "confidence": round(random.uniform(0.75, 0.98), 2),
    }


def draw_shape(draw, primitive: str, w: int, h: int, fill: tuple[int, int, int]) -> None:
    margin = int(min(w, h) * 0.12)
    cx, cy = w // 2, h // 2
    if primitive == "box":
        lw = random.randint(margin, w // 2)
        lh = random.randint(margin, h // 2)
        x0, y0 = cx - lw // 2, cy - lh // 2
        draw.rectangle([x0, y0, x0 + lw, y0 + lh], fill=fill, outline=(30, 30, 30), width=2)
        # 简单透视边
        off = lw // 5
        draw.polygon(
            [(x0, y0), (x0 + off, y0 - off), (x0 + lw + off, y0 - off), (x0 + lw, y0)],
            fill=tuple(max(0, c - 35) for c in fill),
        )
    elif primitive == "cylinder":
        rx = random.randint(margin, w // 3)
        ry = max(8, rx // 3)
        top = cy - random.randint(margin, h // 3)
        bot = cy + random.randint(margin, h // 3)
        draw.ellipse([cx - rx, top - ry, cx + rx, top + ry], fill=fill, outline=(30, 30, 30), width=2)
        draw.rectangle([cx - rx, top, cx + rx, bot], fill=fill)
        draw.ellipse([cx - rx, bot - ry, cx + rx, bot + ry], fill=tuple(max(0, c - 25) for c in fill), outline=(30, 30, 30), width=2)
    elif primitive == "cone":
        rx = random.randint(margin, w // 3)
        top = cy - random.randint(margin, h // 3)
        bot = cy + random.randint(margin, h // 3)
        draw.polygon([(cx, top), (cx - rx, bot), (cx + rx, bot)], fill=fill, outline=(30, 30, 30))
        draw.ellipse([cx - rx, bot - rx // 4, cx + rx, bot + rx // 4], fill=tuple(max(0, c - 30) for c in fill), outline=(30, 30, 30), width=2)
    elif primitive == "sphere":
        r = random.randint(margin, min(w, h) // 3)
        bbox = [cx - r, cy - r, cx + r, cy + r]
        draw.ellipse(bbox, fill=fill, outline=(30, 30, 30), width=2)
        hl = r // 3
        draw.ellipse([cx - r // 3, cy - r // 3, cx - r // 3 + hl, cy - r // 3 + hl], fill=(255, 255, 255, 80))


def render_png(path: Path, primitive: str, size: int) -> None:
    try:
        from PIL import Image, ImageDraw
    except ImportError as exc:
        raise SystemExit("需要 Pillow：pip install pillow") from exc

    bg = (
        random.randint(210, 245),
        random.randint(210, 245),
        random.randint(215, 250),
    )
    fill = (
        random.randint(40, 200),
        random.randint(40, 200),
        random.randint(40, 200),
    )
    img = Image.new("RGB", (size, size), bg)
    draw = ImageDraw.Draw(img, "RGBA")
    angle = random.uniform(-25, 25)
    if abs(angle) > 1:
        img = img.rotate(angle, resample=Image.BICUBIC, fillcolor=bg)
        draw = ImageDraw.Draw(img, "RGBA")
    draw_shape(draw, primitive, size, size, fill)
    path.parent.mkdir(parents=True, exist_ok=True)
    img.save(path, format="PNG")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--per-type", type=int, default=15, help="每种基本体样本数")
    parser.add_argument("--size", type=int, default=512, help="PNG 边长像素")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    random.seed(args.seed)
    IMAGES_DIR.mkdir(parents=True, exist_ok=True)

    rows: list[dict] = []
    for primitive in ("box", "cylinder", "cone", "sphere"):
        for i in range(args.per_type):
            spec = sample_spec(primitive)
            name = f"{primitive}_{i:03d}.png"
            rel = f"images/{name}"
            render_png(IMAGES_DIR / name, primitive, args.size)
            rows.append(
                {
                    "instruction": INSTRUCTION,
                    "input": rel,
                    "output": json.dumps(spec, ensure_ascii=False, separators=(",", ":")),
                }
            )

    DATASET_PATH.write_text(
        "\n".join(json.dumps(r, ensure_ascii=False) for r in rows) + "\n",
        encoding="utf-8",
    )
    print(f"wrote {len(rows)} samples -> {DATASET_PATH}")
    print(f"images -> {IMAGES_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
