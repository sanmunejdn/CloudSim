# Split Widget/source/OsgWidget.cpp into OsgScene.cpp (core) and fragments for OsgWidget.cpp (Qt).
import re
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[2]
WIDGET_SRC = ROOT / "Widget" / "source" / "OsgWidget.cpp"
OUT_CORE = ROOT / "OsgWidgetCore" / "source" / "OsgScene.cpp"
OUT_WIDGET = ROOT / "Widget" / "source" / "OsgWidget.cpp.new"

# Functions whose bodies stay in Widget (Qt / signals / controllers / QFile).
STAY_IN_WIDGET = {
    "OsgWidget",
    "OsgWidget",  # destructor handled by name
}

EXCLUDE_NAMES = {
    "OsgWidget",  # constructor
    "~OsgWidget",
    "importModelFile",
    "loadXyzPointCloud",
    "loadAsciiPlyPointCloud",
    "importPointCloudFile",
    "initUi",
    "initViewer",
    "setBackendParent",
    "syncSelectionFromBackend",
    "syncSelectionForBackendId",
    "pointCloudPluginReport",
    "setSelectedPosition",
    "setSelectedRotationEulerDeg",
    "setSelectedColor",
    "pickAxisAtScreenPos",
    "axisToString",
    "clearPointAnnotations",
    "addPointAnnotation",
    "refreshAnnotationTexts",
    "setAnnotationVisible",
    "removeAnnotation",
    "clearAllAnnotations",
    "annotationSnapshots",
    "restoreAnnotations",
    "updatePointPickMarker",
    "clearPointPickMarker",
    "applyColorToStagingGeometry",
    "applyColorToBackendObject",
    "applyColorToActiveBackendObject",
    "eventFilter",
    "captureImportedPointCloudBackend",
    "captureImportedMeshBackend",
    "captureImportedMeshBackendHierarchy",
    "loadPointCloudFromBackendData",
    "loadMeshFromBackendData",
}


def split_functions(text: str):
    # Match top-level function definitions starting at void/bool/osg::/QString etc.
    pattern = re.compile(
        r"(^[a-zA-Z_:][\w:<>&*,\s]*?OsgWidget::(\w+).*?\n(?:(?:^[^{]).*\n)*?\{)",
        re.MULTILINE,
    )
    # Simpler: split on lines like "void OsgWidget::foo"
    lines = text.splitlines(keepends=True)
    chunks = []
    i = 0
    while i < len(lines):
        line = lines[i]
        m = re.match(r"^([a-zA-Z_][\w:<>&*,\s]*?)OsgWidget::(\w+)\s*\(", line)
        if m:
            name = m.group(2)
            start = i
            brace = 0
            seen_open = False
            j = i
            while j < len(lines):
                for ch in lines[j]:
                    if ch == "{":
                        brace += 1
                        seen_open = True
                    elif ch == "}":
                        brace -= 1
                if seen_open and brace == 0:
                    end = j + 1
                    chunks.append((name, "".join(lines[start:end])))
                    i = end
                    break
                j += 1
            else:
                chunks.append((name, "".join(lines[start:])))
                break
            continue
        i += 1
    return chunks


def main():
    text = WIDGET_SRC.read_text(encoding="utf-8")
    # Extract leading anonymous namespace + includes through first OsgWidget:: or first function
    preamble_end = text.find("OsgWidget::OsgWidget")
    preamble = text[:preamble_end]

    rest = text[preamble_end:]
    chunks = split_functions(rest)
    core_parts = [preamble.replace("OsgWidget.h", "OsgScene.h")]
    widget_parts = []

    for name, body in chunks:
        if name in EXCLUDE_NAMES:
            widget_parts.append(body)
        else:
            core_parts.append(body)

    core_text = "".join(core_parts)
    core_text = core_text.replace("OsgWidget::", "OsgScene::")
    core_text = core_text.replace("OsgScene::DragAxis", "OsgScene::DragAxis")
    core_text = re.sub(r"OsgScene::AnnotationSnapshot", "OsgWidget::AnnotationSnapshot", core_text)
    # Fix QString error messages in geode builders
    core_text = core_text.replace("QString* errorMessage", "std::string* errorMessage")
    core_text = core_text.replace(
        '*errorMessage = QStringLiteral("Invalid point buffer in backend data.");',
        '*errorMessage = "Invalid point buffer in backend data.";',
    )
    core_text = core_text.replace(
        '*errorMessage = QStringLiteral("Invalid mesh buffer in backend data.");',
        '*errorMessage = "Invalid mesh buffer in backend data.";',
    )

    OUT_CORE.parent.mkdir(parents=True, exist_ok=True)
    OUT_CORE.write_text(core_text, encoding="utf-8")

    widget_new = "".join(widget_parts)
    OUT_WIDGET.write_text(widget_new, encoding="utf-8")
    print("Wrote", OUT_CORE, "and", OUT_WIDGET, "chunks", len(chunks))


if __name__ == "__main__":
    main()
