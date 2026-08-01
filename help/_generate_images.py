# -*- coding: utf-8 -*-
"""Generate high-DPI PNG schematics for CloudSim help."""
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

OUT = Path(r"d:\Project\VSprogram\CGAL5.5.2\CloudSim\help\images")
OUT.mkdir(parents=True, exist_ok=True)

# 逻辑坐标按 1x，输出按 SCALE 放大，避免帮助窗缩放后发糊
SCALE = 3

TEAL = (15, 118, 110)
INK = (17, 24, 39)
MUTED = (75, 85, 99)
LINE = (156, 163, 175)
BG = (248, 250, 252)
WHITE = (255, 255, 255)
BLUE = (37, 99, 235)
TEAL_SOFT = (204, 251, 241)
BLUE_SOFT = (239, 246, 255)
PANEL = (243, 244, 246)

_FONT_CACHE = {}


def font(size_logical):
    px = int(round(size_logical * SCALE))
    if px in _FONT_CACHE:
        return _FONT_CACHE[px]
    candidates = [
        r"C:\Windows\Fonts\msyh.ttc",
        r"C:\Windows\Fonts\msyhbd.ttc",
        r"C:\Windows\Fonts\segoeui.ttf",
        r"C:\Windows\Fonts\arial.ttf",
        "msyh.ttc",
        "segoeui.ttf",
    ]
    for name in candidates:
        try:
            f = ImageFont.truetype(name, px)
            _FONT_CACHE[px] = f
            return f
        except OSError:
            continue
    f = ImageFont.load_default()
    _FONT_CACHE[px] = f
    return f


def S(v):
    return int(round(v * SCALE))


def new_img(w, h):
    im = Image.new("RGB", (S(w), S(h)), BG)
    return im, ImageDraw.Draw(im)


def rounded(draw, xy, fill, outline=LINE, width=2, radius=8):
    x0, y0, x1, y1 = xy
    draw.rounded_rectangle(
        (S(x0), S(y0), S(x1), S(y1)),
        radius=S(radius),
        fill=fill,
        outline=outline,
        width=max(1, S(width)),
    )


def text(draw, xy, s, fill, size=14):
    draw.text((S(xy[0]), S(xy[1])), s, fill=fill, font=font(size))


def line(draw, pts, fill, width=2):
    scaled = [(S(x), S(y)) for x, y in pts]
    draw.line(scaled, fill=fill, width=max(1, S(width)))


def ellipse(draw, xy, fill=None, outline=None, width=2):
    x0, y0, x1, y1 = xy
    draw.ellipse(
        (S(x0), S(y0), S(x1), S(y1)),
        fill=fill,
        outline=outline,
        width=max(1, S(width)) if outline else 0,
    )


def polygon(draw, pts, fill, outline=None, width=2):
    scaled = [(S(x), S(y)) for x, y in pts]
    draw.polygon(scaled, fill=fill, outline=outline)
    if outline:
        line(draw, pts + [pts[0]], outline, width)


def save(im, name):
    path = OUT / name
    # optimize=False 保留清晰边缘
    im.save(path, "PNG", optimize=False)
    print("wrote", path.name, im.size)


def ui_layout():
    im, d = new_img(960, 480)
    rounded(d, (24, 48, 936, 456), (238, 242, 247), TEAL, 3, 10)
    rounded(d, (24, 48, 936, 100), TEAL, TEAL, 3, 10)
    text(d, (44, 64), "Workspace modes / Top bar", WHITE, 18)
    rounded(d, (40, 116, 220, 440), PANEL, LINE, 2, 8)
    text(d, (58, 140), "Units tree", MUTED, 15)
    text(d, (52, 175), "Object tree", INK, 20)
    text(d, (52, 210), "• Doc root", MUTED, 14)
    text(d, (52, 238), "• Mesh / Robot", MUTED, 14)
    rounded(d, (240, 116, 700, 360), WHITE, TEAL, 3, 8)
    text(d, (400, 210), "3D Viewport", TEAL, 28)
    text(d, (410, 255), "Central view", MUTED, 16)
    rounded(d, (240, 376, 700, 440), TEAL_SOFT, TEAL, 2, 8)
    text(d, (420, 398), "Run log", TEAL, 18)
    rounded(d, (720, 116, 920, 440), PANEL, LINE, 2, 8)
    text(d, (740, 150), "Workspace", MUTED, 15)
    text(d, (740, 190), "Props / Sim", INK, 18)
    text(d, (740, 230), "Plugins / AI", INK, 18)
    text(d, (740, 270), "AI Assistant", MUTED, 14)
    text(d, (340, 14), "CloudSim main window", MUTED, 18)
    save(im, "ui-layout.png")


def projects():
    im, d = new_img(880, 360)
    rounded(d, (48, 100, 220, 240), WHITE, TEAL, 3, 10)
    text(d, (100, 145), ".pcp", TEAL, 28)
    text(d, (85, 190), "Package", MUTED, 16)
    text(d, (250, 155), "→", MUTED, 28)
    rounded(d, (300, 70, 540, 270), WHITE, LINE, 3, 10)
    text(d, (350, 120), "objects[]", INK, 18)
    text(d, (350, 160), "edges / robot", MUTED, 16)
    text(d, (350, 195), "sidecars", MUTED, 16)
    text(d, (350, 230), "camera", MUTED, 16)
    text(d, (570, 155), "→", MUTED, 28)
    rounded(d, (630, 100, 820, 240), TEAL_SOFT, TEAL, 3, 10)
    text(d, (690, 145), "Save", TEAL, 26)
    text(d, (665, 190), "Open / Save", MUTED, 16)
    text(d, (360, 28), "Project I/O", MUTED, 18)
    save(im, "projects.png")


def view3d():
    im, d = new_img(920, 400)
    rounded(d, (36, 56, 560, 360), WHITE, TEAL, 3, 10)
    ellipse(d, (170, 120, 330, 280), outline=TEAL, width=4)
    line(d, [(250, 120), (250, 280)], TEAL, 3)
    line(d, [(170, 200), (330, 200)], BLUE, 3)
    text(d, (170, 310), "Gizmo Local / World", MUTED, 16)
    rounded(d, (600, 56, 880, 360), PANEL, LINE, 2, 10)
    text(d, (630, 90), "View modes", TEAL, 20)
    for i, t in enumerate(["View", "Object Select", "Point Pick", "Line Pick", "Face Pick"]):
        text(d, (630, 140 + i * 36), f"• {t}", INK, 17)
    text(d, (360, 18), "3D interaction", MUTED, 18)
    save(im, "view-3d.png")


def import_assets():
    im, d = new_img(920, 340)
    for i, (title, sub) in enumerate([("CAD", "STEP / STL"), ("Cloud", "PLY / LAS"), ("URDF", "Robot")]):
        x0 = 40 + i * 200
        rounded(d, (x0, 90, x0 + 160, 230), WHITE, LINE, 3, 10)
        text(d, (x0 + 50, 130), title, INK, 22)
        text(d, (x0 + 35, 175), sub, MUTED, 15)
    text(d, (650, 145), "→", MUTED, 28)
    rounded(d, (710, 80, 880, 240), TEAL_SOFT, TEAL, 3, 10)
    text(d, (755, 130), "Scene", TEAL, 24)
    text(d, (735, 175), "Document", MUTED, 16)
    text(d, (370, 28), "Import assets", MUTED, 18)
    save(im, "import.png")


def geometry():
    im, d = new_img(940, 400)
    rounded(d, (24, 56, 220, 360), PANEL, LINE, 2, 10)
    text(d, (48, 90), "Feature tree", TEAL, 18)
    for i, t in enumerate(["Sketch", "Pad", "Fillet", "Pattern"]):
        text(d, (48, 140 + i * 40), t, INK, 17)
    rounded(d, (250, 56, 640, 360), WHITE, TEAL, 3, 10)
    polygon(d, [(320, 280), (520, 280), (460, 130), (360, 130)], TEAL_SOFT, TEAL, 3)
    text(d, (370, 310), "Sketch → Solid", MUTED, 17)
    rounded(d, (670, 56, 910, 360), BLUE_SOFT, BLUE, 3, 10)
    text(d, (710, 100), "Ribbon", BLUE, 20)
    for i, t in enumerate(["Line / Arc", "Constraint", "Extrude", "Sweep"]):
        text(d, (710, 155 + i * 40), t, INK, 17)
    text(d, (360, 18), "Geometric modeling", MUTED, 18)
    save(im, "geometry.png")


def drawing():
    im, d = new_img(920, 400)
    rounded(d, (40, 56, 560, 360), WHITE, LINE, 3, 10)
    rounded(d, (80, 100, 230, 300), (248, 250, 252), TEAL, 2, 8)
    rounded(d, (260, 100, 410, 300), (248, 250, 252), TEAL, 2, 8)
    rounded(d, (440, 120, 530, 220), TEAL_SOFT, TEAL, 2, 8)
    text(d, (125, 320), "Front", MUTED, 15)
    text(d, (310, 320), "Top", MUTED, 15)
    text(d, (460, 245), "Iso", MUTED, 15)
    rounded(d, (600, 80, 880, 340), PANEL, LINE, 2, 10)
    text(d, (640, 130), "Dims / Notes", TEAL, 18)
    text(d, (640, 180), "SVG / DXF / PDF", INK, 16)
    text(d, (640, 220), "Layers", INK, 16)
    text(d, (340, 18), "Engineering drawing", MUTED, 18)
    save(im, "drawing.png")


def robot():
    im, d = new_img(920, 400)
    line(d, [(140, 320), (220, 210)], TEAL, 14)
    line(d, [(220, 210), (360, 150)], TEAL, 11)
    line(d, [(360, 150), (450, 110)], BLUE, 8)
    for c, r, col in [((140, 320), 18, TEAL), ((220, 210), 14, INK), ((360, 150), 12, INK), ((450, 110), 10, BLUE)]:
        ellipse(d, (c[0] - r, c[1] - r, c[0] + r, c[1] + r), fill=col)
    rounded(d, (520, 70, 880, 350), PANEL, LINE, 2, 10)
    text(d, (560, 120), "PTP / LINE / ARC", TEAL, 18)
    for i, t in enumerate(["Tool / User frame", "TCP drag teach", "Brand export", "Collision"]):
        text(d, (560, 170 + i * 36), t, INK, 16)
    text(d, (360, 18), "Robot simulation", MUTED, 18)
    save(im, "robot.png")


def trajectory():
    im, d = new_img(920, 340)
    rounded(d, (40, 70, 280, 270), WHITE, LINE, 3, 10)
    text(d, (100, 140), "CAD / Mesh", INK, 20)
    text(d, (120, 185), "Feature", MUTED, 16)
    text(d, (310, 155), "→", MUTED, 28)
    pts = [(380, 200), (460, 100), (560, 220), (660, 110), (800, 160)]
    line(d, pts, TEAL, 4)
    for p in pts:
        ellipse(d, (p[0] - 7, p[1] - 7, p[0] + 7, p[1] + 7), fill=TEAL)
    text(d, (480, 280), "Edit ops → Path plan", MUTED, 16)
    text(d, (350, 24), "Trajectory pipeline", MUTED, 18)
    save(im, "trajectory.png")


def process_flow():
    im, d = new_img(960, 340)
    nodes = [
        (40, "Start", TEAL_SOFT, TEAL),
        (220, "Station", WHITE, LINE),
        (400, "Buffer", WHITE, LINE),
        (580, "Inspect", WHITE, LINE),
        (760, "End", BLUE_SOFT, BLUE),
    ]
    for x, name, fill, outline in nodes:
        rounded(d, (x, 110, x + 130, 200), fill, outline, 3, 10)
        text(d, (x + 30, 140), name, outline if outline != LINE else INK, 17)
    for x in (180, 360, 540, 720):
        text(d, (x, 140), "→", MUTED, 22)
    text(d, (360, 250), "DES · Gantt · CSV", MUTED, 17)
    text(d, (390, 30), "Process flow", MUTED, 18)
    save(im, "process-flow.png")


def pointcloud():
    im, d = new_img(880, 360)
    for i in range(60):
        x = 90 + (i % 10) * 22
        y = 100 + (i // 10) * 22
        col = TEAL if i % 3 else BLUE
        ellipse(d, (x - 4, y - 4, x + 4, y + 4), fill=col)
    text(d, (130, 260), "Point cloud", MUTED, 16)
    text(d, (380, 160), "→", MUTED, 28)
    rounded(d, (440, 90, 820, 270), WHITE, TEAL, 3, 10)
    text(d, (540, 135), "ICP / SDF", TEAL, 22)
    text(d, (530, 180), "VCG remesh", INK, 17)
    text(d, (540, 220), "Export PLY", INK, 17)
    text(d, (320, 24), "Point cloud & mesh", MUTED, 18)
    save(im, "pointcloud.png")


def ai():
    im, d = new_img(880, 360)
    rounded(d, (40, 70, 380, 300), WHITE, LINE, 3, 10)
    text(d, (70, 110), "User", MUTED, 15)
    rounded(d, (70, 145, 350, 210), PANEL, LINE, 2, 8)
    text(d, (90, 165), "Create a pad 50mm…", INK, 16)
    text(d, (420, 160), "→", MUTED, 28)
    rounded(d, (500, 70, 840, 300), TEAL_SOFT, TEAL, 3, 10)
    text(d, (580, 120), "AI Domains", TEAL, 20)
    text(d, (555, 170), "feature.compose", INK, 16)
    text(d, (575, 210), "trajectory.*", INK, 16)
    text(d, (580, 250), "process.flow", INK, 16)
    text(d, (360, 24), "AI assistant", MUTED, 18)
    save(im, "ai.png")


def settings():
    im, d = new_img(820, 320)
    rounded(d, (40, 80, 250, 250), WHITE, TEAL, 3, 10)
    text(d, (105, 130), "Theme", TEAL, 22)
    text(d, (85, 175), "Light / Dark", MUTED, 16)
    rounded(d, (290, 80, 500, 250), WHITE, BLUE, 3, 10)
    text(d, (340, 130), "Language", BLUE, 22)
    text(d, (345, 175), "EN / 中文", MUTED, 16)
    rounded(d, (540, 80, 780, 250), WHITE, LINE, 3, 10)
    text(d, (610, 130), "Mode", INK, 22)
    text(d, (580, 175), "Ctrl+1 … 4", MUTED, 16)
    text(d, (350, 28), "Settings", MUTED, 18)
    save(im, "settings.png")


def scene():
    im, d = new_img(880, 360)
    rounded(d, (40, 56, 320, 320), PANEL, LINE, 2, 10)
    text(d, (70, 90), "▾ Doc", TEAL, 18)
    text(d, (80, 140), "• Mesh_01", INK, 16)
    text(d, (80, 175), "• Robot", INK, 16)
    text(d, (110, 210), "• Link_1", MUTED, 15)
    text(d, (80, 250), "• Frame", INK, 16)
    rounded(d, (360, 56, 840, 320), WHITE, TEAL, 3, 10)
    text(d, (400, 100), "Property panel", TEAL, 20)
    for i, t in enumerate(["Pose / Rotation", "Color", "Follow target"]):
        text(d, (400, 155 + i * 40), t, INK, 17)
    text(d, (300, 18), "Scene tree & properties", MUTED, 18)
    save(im, "scene.png")


def labeling():
    im, d = new_img(880, 340)
    rounded(d, (40, 70, 380, 270), WHITE, LINE, 3, 10)
    text(d, (110, 120), "Click / Brush", INK, 20)
    text(d, (115, 165), "Lasso / Erase", INK, 18)
    text(d, (120, 210), "Export PLY+NPY", MUTED, 15)
    text(d, (420, 150), "→", MUTED, 28)
    rounded(d, (490, 70, 840, 270), BLUE_SOFT, BLUE, 3, 10)
    text(d, (590, 130), "PointNet", BLUE, 24)
    text(d, (560, 185), "Prelabel / Train", INK, 17)
    text(d, (350, 24), "Labeling & DL", MUTED, 18)
    save(im, "labeling.png")


def plc_camera():
    im, d = new_img(880, 340)
    rounded(d, (40, 80, 340, 260), WHITE, TEAL, 3, 10)
    text(d, (145, 130), "PLC", TEAL, 26)
    text(d, (95, 185), "EIP / Modbus", INK, 17)
    rounded(d, (400, 80, 840, 260), WHITE, BLUE, 3, 10)
    text(d, (540, 130), "Camera", BLUE, 26)
    text(d, (470, 185), "Hik / Mech / Hand-eye", INK, 17)
    text(d, (300, 28), "PLC & industrial camera", MUTED, 18)
    save(im, "plc-camera.png")


def geometry_plugin():
    im, d = new_img(880, 340)
    rounded(d, (40, 80, 340, 260), WHITE, LINE, 3, 10)
    text(d, (110, 140), "Discretize", INK, 22)
    text(d, (105, 190), "STEP / B-rep", MUTED, 16)
    text(d, (380, 150), "→", MUTED, 28)
    rounded(d, (450, 80, 840, 260), TEAL_SOFT, TEAL, 3, 10)
    text(d, (560, 140), "Intersect", TEAL, 22)
    text(d, (530, 190), "Tube / Ribbon", INK, 17)
    text(d, (290, 28), "Geometry analysis plugin", MUTED, 18)
    save(im, "geometry-plugin.png")


def overview():
    im, d = new_img(960, 380)
    domains = [
        (40, "3D", TEAL),
        (220, "Robot", BLUE),
        (400, "CAD", TEAL),
        (580, "Process", BLUE),
        (760, "AI", TEAL),
    ]
    for x, name, col in domains:
        rounded(d, (x, 110, x + 150, 260), WHITE, col, 3, 10)
        text(d, (x + 40, 165), name, col, 24)
    text(d, (300, 40), "CloudSim capability domains", MUTED, 18)
    text(d, (280, 300), "Simulation · Modeling · Plugins", MUTED, 16)
    save(im, "overview.png")


if __name__ == "__main__":
    ui_layout()
    projects()
    view3d()
    import_assets()
    geometry()
    drawing()
    robot()
    trajectory()
    process_flow()
    pointcloud()
    ai()
    settings()
    scene()
    labeling()
    plc_camera()
    geometry_plugin()
    overview()
    print("total", len(list(OUT.glob("*.png"))), "SCALE=", SCALE)
