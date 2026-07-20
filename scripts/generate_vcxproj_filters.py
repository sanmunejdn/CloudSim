#!/usr/bin/env python3
"""Generate / refresh .vcxproj.filters with inc/src top-level and functional sub-filters."""

from __future__ import annotations

import re
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NS = "{http://schemas.microsoft.com/developer/msbuild/2003}"
ITEM_TAGS = ("ClInclude", "ClCompile", "QtMoc", "QtUic", "QtRcc", "None", "ResourceCompile", "CustomBuild")

FILTER_INC = "{93995380-89BD-4b04-88EB-625FBE52EBFB}"
FILTER_SRC = "{4FC737F1-C7A5-4376-A066-2A32D752A2FF}"


def new_guid() -> str:
    return "{" + str(uuid.uuid4()).upper() + "}"


def basename(path: str) -> str:
    return path.replace("/", "\\").split("\\")[-1]


def functional_subfolder(name: str, full_lower: str) -> str:
    n = name.lower()
    rules: list[tuple[str, str]] = [
        ("pch", "pch"),
        ("_global", "Global"),
        ("global.h", "Global"),
        ("mainwindow", "MainWindow"),
        ("documentpage", "Document"),
        ("documenthost", "Document"),
        ("documentimport", "Document"),
        ("documenthostaccess", "Document"),
        ("widgetdocumentaccess", "Document"),
        ("osgwidget", "OsgWidget"),
        ("osgscene", "OsgWidget"),
        ("graphicswindow", "OsgWidget"),
        ("qwidgetviewer", "OsgWidget"),
        ("qtkeyboard", "OsgWidget"),
        ("objecttransform", "PickGizmo"),
        ("pointpick", "PickGizmo"),
        ("meshedgeface", "PickGizmo"),
        ("selectionoperation", "PickGizmo"),
        ("robottcpdrag", "PickGizmo"),
        ("backendscene", "SceneFacade"),
        ("backendfollowreverse", "SceneFacade"),
        ("ibackendscene", "SceneFacade"),
        ("osgwidgetscenebridge", "SceneFacade"),
        ("backendfileimport", "Backend"),
        ("backendproject", "Backend"),
        ("backendhierarchy", "Backend"),
        ("backendvisual", "Backend"),
        ("backendregistry", "Backend"),
        ("backenddata", "Backend"),
        ("backendproperty", "Backend"),
        ("backendobject", "Backend"),
        ("meshbackend", "Backend"),
        ("pointcloudbackend", "Backend"),
        ("followattachment", "Backend"),
        ("projectpackage", "ProjectIo"),
        ("annotationproject", "ProjectIo"),
        ("robotplan", "Robot"),
        ("robotprogram", "Robot"),
        ("robotproject", "Robot"),
        ("robotscene", "Robot"),
        ("robotinstruction", "Robot"),
        ("robotsimulation", "Robot"),
        ("robotaxis", "Robot"),
        ("robotframe", "Robot"),
        ("robotcoordinate", "Robot"),
        ("robotmatrix", "Robot"),
        ("robotteach", "Robot"),
        ("urdf", "Urdf"),
        ("plugin", "Plugin"),
        ("aicommand", "Ai"),
        ("aibackend", "Ai"),
        ("aiwidget", "Ai"),
        ("aiassistant", "Ai"),
        ("hierarchymesh", "Import"),
        ("importfacade", "Import"),
        ("importcapture", "Import"),
        ("eventhub", "Core"),
        ("coretypes", "Core"),
        ("coreevents", "Core"),
        ("idataservice", "Core"),
        ("irenderview", "Core"),
        ("idocumentscope", "Core"),
        ("icloudsim", "Core"),
        ("cloudsimcore", "Core"),
        ("dataserviceadapter", "adapters"),
        ("osgrenderviewadapter", "adapters"),
        ("robotserviceadapter", "adapters"),
        ("hostrenderview", "Render"),
        ("renderview", "Render"),
        ("applicationcontext", "App"),
        ("cloudsimhostexport", "App"),
        ("cloudsimcoreexport", "App"),
        ("bootstrap", "App"),
        ("runlogger", "Infra"),
        ("runinfo", "Infra"),
        ("jobsystem", "Async"),
        ("progressmanager", "Async"),
        ("devicepage", "Device"),
        ("geometryengine", "Geometry"),
        ("rigidtransform", "Geometry"),
        ("toolkinematics", "Geometry"),
        ("propertycore", "PropertyCore"),
        ("propertytypes", "PropertyCore"),
        ("simulationcommand", "Simulation"),
        ("applicationstyle", "UiStyle"),
        ("geometry_base64", "MeshIo"),
        ("meshbackenddata", "MeshIo"),
        ("pointcloudalgorithm", "PointCloud"),
        ("backendvisualsync", "Sync"),
        ("selectionservice", "Selection"),
        ("selectionstate", "Selection"),
        ("litmesh", "Material"),
        ("helloplugin", "Plugin"),
        ("cloudsimplugin", "Plugin"),
    ]
    for key, folder in rules:
        if key in n or key in full_lower:
            return folder
    if "adapter" in n:
        return "adapters"
    if "export" in n or "import" in n:
        return "Io"
    return "Common"


def resolve_filter(include_path: str) -> str:
    p = include_path.replace("/", "\\")
    pl = p.lower()
    b = basename(p)
    b_lower = b.lower()

    if "bin\\sdk\\qtpropertybrowser" in pl or "qtpropertybrowser" in pl:
        return r"src\ThirdParty\qtpropertybrowser"
    if "dxflib" in pl or "dxfsolution" in pl:
        return r"src\ThirdParty\dxflib"

    if "cloudsimpluginhost" in pl:
        root = "inc" if ("\\inc\\" in pl or b_lower.endswith(".h")) and "\\source\\" not in pl else "src"
        sub = functional_subfolder(b, pl)
        return f"{root}\\PluginHost" if sub == "Common" else f"{root}\\PluginHost\\{sub}"

    if "propertycore" in pl:
        root = "inc" if "\\inc\\" in pl or "\\inc\\propertycore" in pl else "src"
        return f"{root}\\PropertyCore"

    if "..\\host\\cloudsimhost" in pl or "..\\..\\host\\cloudsimhost" in pl:
        sub = functional_subfolder(b, pl)
        return r"inc\HostRef" if sub == "Common" else f"inc\\HostRef\\{sub}"

    if "..\\ui\\widget" in pl or "..\\..\\ui\\widget" in pl:
        sub = functional_subfolder(b, pl)
        if sub in ("OsgWidget", "PickGizmo", "SceneFacade"):
            return f"src\\{sub}"
        return r"src\WidgetBorrowed" if sub == "Common" else f"src\\WidgetBorrowed\\{sub}"

    if re.search(r"(?:^|[\\/])inc[\\/]", p, re.I):
        m = re.search(r"inc[\\/]([^\\/]+)[\\/]", p, re.I)
        if m and m.group(1).lower() not in ("inc",):
            subdir = m.group(1)
            if subdir.lower() == "adapters":
                return r"inc\adapters"
            return f"inc\\{subdir}"
        sub = functional_subfolder(b, pl)
        if sub == "Common" and b_lower.startswith("backend"):
            sub = "Backend"
        return r"inc" if sub == "Common" else f"inc\\{sub}"

    if re.search(r"(?:^|[\\/])source[\\/]", p, re.I):
        m = re.search(r"source[\\/]([^\\/]+)[\\/]", p, re.I)
        if m:
            subdir = m.group(1)
            if subdir.lower() == "adapters":
                return r"src\adapters"
            return f"src\\{subdir}"
        sub = functional_subfolder(b, pl)
        if b_lower == "pch.cpp":
            return r"src\pch"
        return r"src" if sub == "Common" else f"src\\{sub}"

    if b_lower.endswith((".cpp", ".c", ".cc", ".cxx")):
        sub = functional_subfolder(b, pl)
        return r"src" if sub == "Common" else f"src\\{sub}"

    sub = functional_subfolder(b, pl)
    return r"inc" if sub == "Common" else f"inc\\{sub}"


def ensure_filter_tree(filters: dict[str, str], filter_path: str) -> None:
    parts = filter_path.split("\\")
    for i in range(1, len(parts) + 1):
        sub = "\\".join(parts[:i])
        if sub not in filters:
            filters[sub] = new_guid()


def collect_items(vcxproj: Path) -> dict[str, list[tuple[str, str]]]:
    tree = ET.parse(vcxproj)
    root = tree.getroot()
    items: dict[str, list[tuple[str, str]]] = {tag: [] for tag in ITEM_TAGS}
    for tag in ITEM_TAGS:
        for elem in root.findall(f".//{NS}{tag}"):
            inc = elem.get("Include")
            if inc:
                items[tag].append((inc, resolve_filter(inc)))
    return items


def write_filters(vcxproj: Path, items: dict[str, list[tuple[str, str]]]) -> None:
    filters_path = vcxproj.with_suffix(".vcxproj.filters")
    filter_map: dict[str, str] = {
        "inc": FILTER_INC,
        "src": FILTER_SRC,
    }

    all_filters: set[str] = set()
    for tag_list in items.values():
        for _, f in tag_list:
            all_filters.add(f)
            ensure_filter_tree(filter_map, f)

    lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        '<Project ToolsVersion="4.0" xmlns="http://schemas.microsoft.com/developer/msbuild/2003">',
        "  <ItemGroup>",
    ]

    def filter_sort_key(f: str) -> tuple:
        return tuple(f.split("\\"))

    for f in sorted(filter_map.keys(), key=filter_sort_key):
        if f in ("inc", "src"):
            ext = (
                "h;hh;hpp;hxx;h++;hm;inl;inc;ipp;xsd"
                if f == "inc"
                else "cpp;c;cc;cxx;c++;cppm;ixx;def;odl;idl;hpj;bat;asm;asmx"
            )
            lines.append(f'    <Filter Include="{f}">')
            lines.append(f"      <UniqueIdentifier>{filter_map[f]}</UniqueIdentifier>")
            lines.append(f"      <Extensions>{ext}</Extensions>")
            lines.append("    </Filter>")
        else:
            lines.append(f'    <Filter Include="{f}">')
            lines.append(f"      <UniqueIdentifier>{filter_map[f]}</UniqueIdentifier>")
            lines.append("    </Filter>")

    lines.append("  </ItemGroup>")

    for tag in ITEM_TAGS:
        if not items[tag]:
            continue
        lines.append("  <ItemGroup>")
        for inc, filt in sorted(items[tag], key=lambda x: x[0].lower()):
            lines.append(f'    <{tag} Include="{inc}">')
            lines.append(f"      <Filter>{filt}</Filter>")
            lines.append(f"    </{tag}>")
        lines.append("  </ItemGroup>")

    lines.append("</Project>")
    # UTF-8 BOM + CRLF for VS Chinese locale
    payload = ("\r\n".join(lines) + "\r\n").encode("utf-8")
    filters_path.write_bytes(b"\xef\xbb\xbf" + payload)
    print(f"  wrote {filters_path.relative_to(ROOT)}")


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="Generate / refresh .vcxproj.filters")
    ap.add_argument(
        "--only-missing",
        action="store_true",
        help="Only create .filters when the file does not already exist",
    )
    args = ap.parse_args()

    projects = sorted((ROOT / "src").rglob("*.vcxproj"))
    projects = [
        p
        for p in projects
        if ".vs" not in str(p)
        and "ThirdParty" not in p.parts
        and "vcglib" not in {x.lower() for x in p.parts}
        and "bin" not in {x.lower() for x in p.parts}
    ]
    print(f"Processing {len(projects)} projects (only_missing={args.only_missing})...")
    for vcx in projects:
        filters_path = vcx.with_suffix(".vcxproj.filters")
        if args.only_missing and filters_path.exists():
            continue
        print(vcx.relative_to(ROOT))
        items = collect_items(vcx)
        if not any(items[t] for t in ITEM_TAGS):
            print("  (no file items, skip)")
            continue
        write_filters(vcx, items)


if __name__ == "__main__":
    main()
