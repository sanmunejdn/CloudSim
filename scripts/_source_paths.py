"""Shared helpers: enumerate CloudSim product sources (exclude third-party)."""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"

SOURCE_SUFFIXES = {".h", ".hpp", ".hh", ".hxx", ".cpp", ".cxx", ".cc", ".c"}

EXCLUDE_DIR_PARTS = {
    "thirdparty",
    "vcglib",
    "bin",
    ".vs",
    "x64",
    "x86",
    "debug",
    "release",
    "generatedfiles",
}


def is_excluded(path: Path) -> bool:
    parts_lower = {p.lower() for p in path.parts}
    if parts_lower & EXCLUDE_DIR_PARTS:
        return True
    # InstantMeshes vendored trees (keep only thin wrappers under InstantMeshesCore/Lib project roots)
    name = path.name.lower()
    if name.startswith("moc_") or name.startswith("ui_") or name.startswith("qrc_"):
        return True
    pl = str(path).replace("\\", "/").lower()
    if "/instantmeshes/" in pl and "/src/geometry/instantmeshes" not in pl:
        return True
    # Nested InstantMeshes third-party under Geometry modules
    if "instantmeshes/ext" in pl or "instantmeshes/src" in pl:
        # Allow files directly under InstantMeshesCore / InstantMeshesLib module folders
        if "instantmeshescore" not in pl and "instantmesheslib" not in pl:
            return True
    return False


def iter_sources(root: Path | None = None) -> list[Path]:
    base = root or SRC
    out: list[Path] = []
    for p in base.rglob("*"):
        if not p.is_file():
            continue
        if p.suffix.lower() not in SOURCE_SUFFIXES:
            continue
        if is_excluded(p):
            continue
        out.append(p)
    return sorted(out)


def project_name_for(path: Path) -> str:
    """Nearest ancestor that contains a .vcxproj, else parent of inc/source."""
    cur = path.parent
    while cur != ROOT and cur != cur.parent:
        vcx = list(cur.glob("*.vcxproj"))
        if vcx:
            return vcx[0].stem
        cur = cur.parent
    # Fallback: src/<Domain>/<Module>/...
    try:
        rel = path.relative_to(SRC)
        if len(rel.parts) >= 2:
            return rel.parts[1]
    except ValueError:
        pass
    return path.parent.name


def guard_macro(path: Path) -> str:
    project = "".join(c if c.isalnum() else "" for c in project_name_for(path)).upper()
    stem = "".join(c if c.isalnum() else "_" for c in path.stem).upper()
    stem = "_".join(filter(None, stem.split("_")))
    return f"{project}_{stem}_H"
